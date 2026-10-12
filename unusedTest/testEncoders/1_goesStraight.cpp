/*
- Goes pretty straight but without kalman filter
*/
#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "BNO/bno.hpp"

// ═══════════════════════════════════════════════════════════════
//  4-Motor PID + Odometría + compensación lateral
// ═══════════════════════════════════════════════════════════════

BNO bno1;

const uint8_t encUL_A = Pins::kEncoders[1];
const uint8_t encUL_B = Pins::kEncoders[0];
const uint8_t pwmUL   = Pins::kPwmPin[0];
const uint8_t inUL1   = Pins::kUpperMotors[0];
const uint8_t inUL2   = Pins::kUpperMotors[1];

const uint8_t encUR_A = Pins::kEncoders[2];
const uint8_t encUR_B = Pins::kEncoders[3];
const uint8_t pwmUR   = Pins::kPwmPin[1];
const uint8_t inUR1   = Pins::kUpperMotors[2];
const uint8_t inUR2   = Pins::kUpperMotors[3];

const uint8_t encLL_A = Pins::kEncoders[4];
const uint8_t encLL_B = Pins::kEncoders[5];
const uint8_t pwmLL   = Pins::kPwmPin[2];
const uint8_t inLL1   = Pins::kLowerMotors[0];
const uint8_t inLL2   = Pins::kLowerMotors[1];

const uint8_t encLR_A = Pins::kEncoders[6];
const uint8_t encLR_B = Pins::kEncoders[7];
const uint8_t pwmLR   = Pins::kPwmPin[3];
const uint8_t inLR1   = Pins::kLowerMotors[2];
const uint8_t inLR2   = Pins::kLowerMotors[3];

#define FILTER_SIZE 8

struct Motor {
    uint8_t pwmPin, in1, in2;
    volatile unsigned long period_buf[FILTER_SIZE];
    volatile uint8_t       period_idx;
    volatile unsigned long last_pulse_us;
    volatile bool          got_pulse;
    float PPR, Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    int8_t motorDir;
};

Motor UR = { pwmUR, inUR1, inUR2, {}, 0, 0, false, 475.0f, 3.4f, 2.2f, 0.0010f, 0.0f, 0, 0, -1 };
Motor UL = { pwmUL, inUL1, inUL2, {}, 0, 0, false, 482.0f, 2.5f, 2.8f, 0.0022f, 0.0f, 0, 0, +1 };
Motor LL = { pwmLL, inLL1, inLL2, {}, 0, 0, false, 495.0f, 2.1f, 3.9f, 0.0012f, 0.0f, 0, 0, +1 };
Motor LR = { pwmLR, inLR1, inLR2, {}, 0, 0, false, 486.0f, 1.3f, 1.0f, 0.0035f, 0.0f, 0, 0, -1 };

const float TARGET_RPM            = 48.0f;
const float RAMP_STEP             = 1.0f;
const unsigned long RAMP_INTERVAL = 200;
unsigned long lastRampTick        = 0;
bool rampDone                     = false;

const float Ts = 0.05f;
unsigned long last_time = 0;

const float WHEEL_DIAMETER = 0.108f;
const float WHEEL_RADIUS   = WHEEL_DIAMETER / 2.0f;
const float INV_SQRT2      = 0.70710678f;
const float ODOM_LINEAR_SCALE = 2.25f;

float odom_x  = 0.0f;
float odom_y  = 0.0f;
float odom_th = 0.0f;

unsigned long last_odom_time   = 0;
const unsigned long ODOM_PRINT_INTERVAL = 200;
unsigned long last_odom_print  = 0;

// ===== compensación =====
// si se va a la derecha, baja el lado izquierdo
const float kLeftScale  = 1.0f;
const float kRightScale = 1.16f;

inline void pushPeriod(Motor &m, unsigned long p)
{
    m.period_buf[m.period_idx] = p;
    m.period_idx = (m.period_idx + 1) % FILTER_SIZE;
    m.got_pulse  = true;
}

void isrUR_A() { unsigned long n = micros(); unsigned long p = n - UR.last_pulse_us; UR.last_pulse_us = n; if (p > 200) pushPeriod(UR, p); }
void isrUR_B() { unsigned long n = micros(); unsigned long p = n - UR.last_pulse_us; UR.last_pulse_us = n; if (p > 200) pushPeriod(UR, p); }
void isrUL_A() { unsigned long n = micros(); unsigned long p = n - UL.last_pulse_us; UL.last_pulse_us = n; if (p > 200) pushPeriod(UL, p); }
void isrUL_B() { unsigned long n = micros(); unsigned long p = n - UL.last_pulse_us; UL.last_pulse_us = n; if (p > 200) pushPeriod(UL, p); }
void isrLL_A() { unsigned long n = micros(); unsigned long p = n - LL.last_pulse_us; LL.last_pulse_us = n; if (p > 200) pushPeriod(LL, p); }
void isrLL_B() { unsigned long n = micros(); unsigned long p = n - LL.last_pulse_us; LL.last_pulse_us = n; if (p > 200) pushPeriod(LL, p); }
void isrLR_A() { unsigned long n = micros(); unsigned long p = n - LR.last_pulse_us; LR.last_pulse_us = n; if (p > 200) pushPeriod(LR, p); }
void isrLR_B() { unsigned long n = micros(); unsigned long p = n - LR.last_pulse_us; LR.last_pulse_us = n; if (p > 200) pushPeriod(LR, p); }

float measureRPM(Motor &m)
{
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t i = 0; i < FILTER_SIZE; i++) buf[i] = m.period_buf[i];
    unsigned long last = m.last_pulse_us;
    bool has_got = m.got_pulse;
    interrupts();

    if (!has_got) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    unsigned long sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        if (buf[i] > 0) {
            sum += buf[i];
            count++;
        }
    }
    if (count == 0) return 0.0f;

    float avg_period = (float)(sum / count);
    return 60000000.0f / (avg_period * 4.0f * m.PPR);
}

float rpmToRadS(float rpm)
{
    return rpm * 2.0f * PI / 60.0f;
}

void setMotor(Motor &m, float pwm)
{
    pwm *= m.motorDir;

    if (pwm >= 0.0f) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        pwm = -pwm;
    }

    analogWrite(m.pwmPin, (int)constrain(pwm, 0.0f, 255.0f));
}

void pidStep(Motor &m, const char* label)
{
    float rpm   = measureRPM(m);
    float error = m.setpoint - rpm;

    m.integral += error * Ts;
    m.integral  = constrain(m.integral, -200.0f, 200.0f);

    float derivative = (error - m.last_error) / Ts;
    float output     = m.Kp * error + m.Ki * m.integral + m.Kd * derivative;
    output           = constrain(output, 0.0f, 255.0f);
    m.last_error     = error;

    setMotor(m, output);

    Serial.print(label); Serial.print("_rpm:");
    Serial.print(rpm, 2);
    Serial.print(",sp:");
    Serial.print(m.setpoint, 2);
    Serial.print(",out:");
    Serial.print(output, 2);
    Serial.print("  ");
}

void updateOdom(float dt)
{
    float rpm_UL = +measureRPM(UL);
    float rpm_UR = -measureRPM(UR);
    float rpm_LL = -measureRPM(LL);
    float rpm_LR = +measureRPM(LR);

    float w_UL = rpmToRadS(rpm_UL);
    float w_UR = rpmToRadS(rpm_UR);
    float w_LL = rpmToRadS(rpm_LL);
    float w_LR = rpmToRadS(rpm_LR);

    const float k_lin = INV_SQRT2 * WHEEL_RADIUS / 4.0f;

    float vx = ( w_UL - w_UR - w_LL + w_LR) * k_lin;
    float vy = (-w_UL - w_UR + w_LL + w_LR) * k_lin;

    vx *= ODOM_LINEAR_SCALE;
    vy *= ODOM_LINEAR_SCALE;

    bno1.update();
    odom_th = bno1.getYaw();

    float c = cos(odom_th);
    float s = sin(odom_th);

    odom_x += (vx * c - vy * s) * dt;
    odom_y += (vx * s + vy * c) * dt;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    bno1.begin();

    uint8_t encoderPins[] = {
        encUR_A, encUR_B,
        encUL_A, encUL_B,
        encLL_A, encLL_B,
        encLR_A, encLR_B
    };

    for (uint8_t p : encoderPins) pinMode(p, INPUT_PULLUP);

    uint8_t motorPins[] = {
        pwmUR, inUR1, inUR2,
        pwmUL, inUL1, inUL2,
        pwmLL, inLL1, inLL2,
        pwmLR, inLR1, inLR2
    };

    for (uint8_t p : motorPins) pinMode(p, OUTPUT);

    analogWriteResolution(8);
    analogWriteFrequency(pwmUR, 25000);
    analogWriteFrequency(pwmUL, 25000);
    analogWriteFrequency(pwmLL, 25000);
    analogWriteFrequency(pwmLR, 25000);

    attachInterrupt(digitalPinToInterrupt(encUR_A), isrUR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUR_B), isrUR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUL_A), isrUL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUL_B), isrUL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLL_A), isrLL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLL_B), isrLL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLR_A), isrLR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLR_B), isrLR_B, CHANGE);

    last_time       = millis();
    lastRampTick    = millis();
    last_odom_time  = millis();
    last_odom_print = millis();
}

void loop()
{
    unsigned long now = millis();

    if (!rampDone && (now - lastRampTick >= RAMP_INTERVAL)) {
        lastRampTick = now;

        float baseSp = UR.setpoint + RAMP_STEP;
        if (baseSp >= TARGET_RPM) {
            baseSp = TARGET_RPM;
            rampDone = true;
            Serial.println(">>> Rampa completa");
        }

        // compensación: bajar lado izquierdo
        UL.setpoint = baseSp * kLeftScale;
        LL.setpoint = baseSp * kLeftScale;

        UR.setpoint = baseSp * kRightScale;
        LR.setpoint = baseSp * kRightScale;
    }

    if (now - last_time >= (unsigned long)(Ts * 1000.0f))
    {
        last_time += (unsigned long)(Ts * 1000.0f);

        float dt = (now - last_odom_time) / 1000.0f;
        last_odom_time = now;

        updateOdom(dt);

        pidStep(UR, "UR");
        pidStep(UL, "UL");
        pidStep(LL, "LL");
        pidStep(LR, "LR");

        Serial.println();
    }

    if (now - last_odom_print >= ODOM_PRINT_INTERVAL) {
        last_odom_print = now;

        float dist = sqrt(odom_x * odom_x + odom_y * odom_y);

        Serial.print(">>> ODOM  x:");
        Serial.print(odom_x, 4);
        Serial.print(" m  y:");
        Serial.print(odom_y, 4);
        Serial.print(" m  dist:");
        Serial.print(dist, 4);
        Serial.print(" m  th:");
        Serial.print(odom_th * 180.0f / PI, 2);
        Serial.println(" deg");
    }
}