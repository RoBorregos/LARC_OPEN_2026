/*
// UR calibration :: RPMs: 188  (02 Septiembre 2026)


// Calibration    <<<<<UR>>>>>>

//Use this code to calibrate

// Almost last code or test for the Encoders
// Works (4:50 am)
// FINAL gives velocit to the motor

Calibrated A7 440: check
*/

#include <Arduino.h>

const uint8_t encUL_B_Pin = 13;
const uint8_t encUL_A_Pin = 2;
const uint8_t motorPWM    = 9;
const uint8_t motorIN1    = 36;
const uint8_t motorIN2    = 35;

#define FILTER_SIZE 12

volatile unsigned long period_buf[FILTER_SIZE] = {0};
volatile uint8_t       period_idx  = 0;
volatile unsigned long last_pulse_us = 0;
volatile bool          got_pulse   = false;



const float PPR = 188.0f;
const float Ts  = 0.05f;   // 50ms — más estable que 10ms

float Kp = 0.5f;//3.4f;
float Ki = 0.6f;//1.0f;
float Kd = 0.0f;//0.001f;

float setpoint   = 45.0f;

// Feedforward UR: recta PWM = offset + pendiente * rpm.
// Medido EN EL SUELO: 30 rpm ~ 63 PWM, 45 rpm ~ 79 PWM (aire: 45 rpm ~ 56 PWM).
//   FF_SLOPE = (out45 - out30) / 15 ;  FF_OFFSET = out30 - FF_SLOPE * 30
// Se deja un poco por debajo de lo medido (el integrador completa lo que falte).
const float FF_OFFSET = 30.0f;  // PWM
const float FF_SLOPE  = 1.05f;  // PWM por rpm  -> 30 rpm: 61.5, 45 rpm: 77
// Rampa del setpoint efectivo para no arrancar con error grande
const float SP_RAMP = 90.0f;    // rpm/s
float sp_ramped = 0.0f;

float integral   = 0.0f;
float last_error = 0.0f;

// Filtro EMA sobre el RPM medido (suaviza el ruido de cuantizacion del encoder)
const float RPM_ALPHA = 0.2f; // 0=sin cambio, 1=sin filtro
float rpm_filt = 0.0f;

unsigned long last_time = 0;

void pushPeriod(unsigned long p)
{
    period_buf[period_idx] = p;
    period_idx = (period_idx + 1) % FILTER_SIZE;
    got_pulse  = true;
}

void isrA()
{
    unsigned long now = micros();
    unsigned long p   = now - last_pulse_us;
    last_pulse_us     = now;
    if (p > 200UL) pushPeriod(p);  // Ignora rebotes < 200µs
}

void isrB()
{
    unsigned long now = micros();
    unsigned long p   = now - last_pulse_us;
    last_pulse_us     = now;
    if (p > 200UL) pushPeriod(p);  // Ignora rebotes < 200µs
}

float measureRPM()
{
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t i = 0; i < FILTER_SIZE; i++) buf[i] = period_buf[i];
    unsigned long last    = last_pulse_us;
    bool          has_got = got_pulse;
    interrupts();

    if (!has_got) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    // Promedio de períodos válidos
    unsigned long sum   = 0;
    uint8_t       count = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        if (buf[i] > 0) { sum += buf[i]; count++; }
    }
    if (count == 0) return 0.0f;

    float avg_period = (float)(sum / count);
    return 60000000.0f / (avg_period * 4.0f * PPR);
}

void setMotor(float pwm)
{
    if (pwm >= 0.0f) {
        digitalWrite(motorIN1, HIGH);
        digitalWrite(motorIN2, LOW);
    } else {
        digitalWrite(motorIN1, LOW);
        digitalWrite(motorIN2, HIGH);
        pwm = -pwm;
    }
    analogWrite(motorPWM, (int)constrain(pwm, 0.0f, 255.0f));
}

void setup()
{
    Serial.begin(115200);

    pinMode(encUL_A_Pin, INPUT_PULLUP);
    pinMode(encUL_B_Pin, INPUT_PULLUP);
    pinMode(motorIN1, OUTPUT);
    pinMode(motorIN2, OUTPUT);
    pinMode(motorPWM, OUTPUT);

    analogWriteResolution(8);
    analogWriteFrequency(motorPWM, 25000);

    attachInterrupt(digitalPinToInterrupt(encUL_A_Pin), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUL_B_Pin), isrB, CHANGE);

    last_time = millis();
}

void loop()
{
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(Ts * 1000.0f))
    {
        last_time += (unsigned long)(Ts * 1000.0f);

        float rpm_raw = measureRPM();
        rpm_filt      += RPM_ALPHA * (rpm_raw - rpm_filt);
        float rpm     = rpm_filt;

        if (sp_ramped < setpoint)      sp_ramped = min(setpoint, sp_ramped + SP_RAMP * Ts);
        else if (sp_ramped > setpoint) sp_ramped = max(setpoint, sp_ramped - SP_RAMP * Ts);

        float error   = sp_ramped - rpm;
        float ff      = (sp_ramped > 0.0f) ? FF_OFFSET + FF_SLOPE * sp_ramped : 0.0f;

        float derivative       = (error - last_error) / Ts;
        float output_unclamped = ff + Kp * error + Ki * integral + Kd * derivative;

        // Solo integra si el output no esta saturado (anti-windup real)
        if (output_unclamped > 0.0f && output_unclamped < 255.0f) {
            integral += error * Ts;
            // Limite = rango completo de PWM (255) / Ki, para que Ki*integral pueda cubrir todo el output
            integral  = constrain(integral, -255.0f / Ki, 255.0f / Ki);
        }

        float output = constrain(output_unclamped, 0.0f, 255.0f);
        last_error    = error;

        setMotor(output);

        Serial.print("rpm_ul:");    Serial.print(rpm, 2);
        Serial.print(",Setpoint:"); Serial.print(setpoint);
        Serial.print(",output:");   Serial.println(output);
    }
}