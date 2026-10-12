// 4-Motor PID Controller - ramp setpoint 0 to 70 RPM
#include <Arduino.h>
#include "pins.h"

// Pin assignments
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

// Filter
#define FILTER_SIZE 8

// Motor struct
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
    int8_t dir;
};

// Motor instances
Motor UR = { pwmUR, inUR1, inUR2, {}, 0, 0, false, 450.0f, 3.4f, 2.2f,  0.001f,  0.0f, 0, 0, -1 };
Motor UL = { pwmUL, inUL1, inUL2, {}, 0, 0, false, 482.0f, 2.5f, 2.8f,  0.0022f, 0.0f, 0, 0,  1 };
Motor LL = { pwmLL, inLL1, inLL2, {}, 0, 0, false, 495.0f, 2.1f, 3.9f,  0.0012f, 0.0f, 0, 0,  1 };
Motor LR = { pwmLR, inLR1, inLR2, {}, 0, 0, false, 486.0f, 1.3f, 1.0f,  0.0035f, 0.0f, 0, 0, -1 };

// Ramp
const float TARGET_RPM   = 48.0f;  // target RPM
const float RAMP_STEP    = 1.0f;   // RPM increase per tick
const unsigned long RAMP_INTERVAL = 200; // ms between each increment
unsigned long lastRampTick = 0;
bool rampDone = false;

// PID timing
const float   Ts        = 0.05f;
unsigned long last_time = 0;

// Encoder push
inline void pushPeriod(Motor &m, unsigned long p)
{
    m.period_buf[m.period_idx] = p;
    m.period_idx = (m.period_idx + 1) % FILTER_SIZE;
    m.got_pulse  = true;
}

// ISRs
void isrUR_A() { unsigned long n=micros(); unsigned long p=n-UR.last_pulse_us; UR.last_pulse_us=n; if(p>200) pushPeriod(UR,p); }
void isrUR_B() { unsigned long n=micros(); unsigned long p=n-UR.last_pulse_us; UR.last_pulse_us=n; if(p>200) pushPeriod(UR,p); }
void isrUL_A() { unsigned long n=micros(); unsigned long p=n-UL.last_pulse_us; UL.last_pulse_us=n; if(p>200) pushPeriod(UL,p); }
void isrUL_B() { unsigned long n=micros(); unsigned long p=n-UL.last_pulse_us; UL.last_pulse_us=n; if(p>200) pushPeriod(UL,p); }
void isrLL_A() { unsigned long n=micros(); unsigned long p=n-LL.last_pulse_us; LL.last_pulse_us=n; if(p>200) pushPeriod(LL,p); }
void isrLL_B() { unsigned long n=micros(); unsigned long p=n-LL.last_pulse_us; LL.last_pulse_us=n; if(p>200) pushPeriod(LL,p); }
void isrLR_A() { unsigned long n=micros(); unsigned long p=n-LR.last_pulse_us; LR.last_pulse_us=n; if(p>200) pushPeriod(LR,p); }
void isrLR_B() { unsigned long n=micros(); unsigned long p=n-LR.last_pulse_us; LR.last_pulse_us=n; if(p>200) pushPeriod(LR,p); }

// Measure RPM
float measureRPM(Motor &m)
{
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t i = 0; i < FILTER_SIZE; i++) buf[i] = m.period_buf[i];
    unsigned long last    = m.last_pulse_us;
    bool          has_got = m.got_pulse;
    interrupts();

    if (!has_got) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    unsigned long sum = 0;
    uint8_t       count = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        if (buf[i] > 0) { sum += buf[i]; count++; }
    }
    if (count == 0) return 0.0f;

    float avg = (float)(sum / count);
    return 60000000.0f / (avg * 4.0f * m.PPR);
}

// Set motor
void setMotor(Motor &m, float pwm)
{
    pwm *= m.dir;
    if (pwm >= 0.0f) {
        digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);
    } else {
        digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH);
        pwm = -pwm;
    }
    analogWrite(m.pwmPin, (int)constrain(pwm, 0.0f, 255.0f));
}

// PID step
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

    Serial.print(label); Serial.print("_rpm:"); Serial.print(rpm, 2);
    Serial.print(",sp:");  Serial.print(m.setpoint);
    Serial.print(",out:"); Serial.print(output);
    Serial.print("  ");
}

// Setup
void setup()
{
    Serial.begin(115200);

    uint8_t encoderPins[] = { encUR_A, encUR_B, encUL_A, encUL_B,
                               encLL_A, encLL_B, encLR_A, encLR_B };
    for (uint8_t p : encoderPins) pinMode(p, INPUT_PULLUP);

    uint8_t motorPins[] = { pwmUR, inUR1, inUR2,
                             pwmUL, inUL1, inUL2,
                             pwmLL, inLL1, inLL2,
                             pwmLR, inLR1, inLR2 };
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

    last_time      = millis();
    lastRampTick   = millis();
}

// Loop
void loop()
{
    unsigned long now = millis();

    // Setpoint ramp
    if (!rampDone && now - lastRampTick >= RAMP_INTERVAL) {
        lastRampTick = now;
        float newSp = UR.setpoint + RAMP_STEP;
        if (newSp >= TARGET_RPM) {
            newSp  = TARGET_RPM;
            rampDone = true;
            Serial.println(">>> Rampa completa");
        }
        UR.setpoint = newSp;
        UL.setpoint = newSp;
        LL.setpoint = newSp;
        LR.setpoint = newSp;
    }

    // PID loop
    if (now - last_time >= (unsigned long)(Ts * 1000.0f))
    {
        last_time += (unsigned long)(Ts * 1000.0f);

        pidStep(UR, "UR");
        pidStep(UL, "UL");
        pidStep(LL, "LL");
        pidStep(LR, "LR");

        Serial.println();
    }
}