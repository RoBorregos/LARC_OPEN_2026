/*
 * Final encoder and motor PID test.
 *
 * This code measures the RPM of all four motors and uses a PID controller
 * to maintain a desired speed.
 *
 * The same PPR value and PID gains obtained from the Upper Left motor
 * were temporarily applied to all four motors, so the results should not
 * be considered a final individual calibration.
 *
 * The encoder direction is not calculated; only the magnitude of the
 * rotational speed is measured.
 *
 * Note: The Lower Right encoder was not working during this test.
 */

// Last Encoder test (not accurate beacuse we sample the configuration of encoder UL for all motors)
#include <Arduino.h>
#include "pins.h"

class MotorPID
{
public:
    static const uint8_t FILTER_SIZE = 8;

    uint8_t pinIN1, pinIN2, pinPWM;
    uint8_t pinEncA, pinEncB;

    volatile unsigned long period_buf[FILTER_SIZE];
    volatile uint8_t       period_idx;
    volatile unsigned long last_pulse_us;
    volatile bool          got_pulse;

    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    float output;
    float rpm;

    const float PPR = 475.0f;
    const float Ts  = 0.05f;

    MotorPID(uint8_t in1, uint8_t in2, uint8_t pwm,
             uint8_t encA, uint8_t encB,
             float kp = 3.4f, float ki = 2.2f, float kd = 0.001f)
        : pinIN1(in1), pinIN2(in2), pinPWM(pwm),
          pinEncA(encA), pinEncB(encB),
          Kp(kp), Ki(ki), Kd(kd),
          setpoint(0), integral(0), last_error(0), output(0), rpm(0),
          period_idx(0), last_pulse_us(0), got_pulse(false)
    {
        for (uint8_t i = 0; i < FILTER_SIZE; i++) period_buf[i] = 0;
    }

    void begin()
    {
        pinMode(pinIN1,  OUTPUT);
        pinMode(pinIN2,  OUTPUT);
        pinMode(pinPWM,  OUTPUT);
        pinMode(pinEncA, INPUT_PULLUP);
        pinMode(pinEncB, INPUT_PULLUP);
        analogWriteFrequency(pinPWM, 25000);
    }

    // Llamar desde cada ISR
    void onPulse()
    {
        unsigned long now = micros();
        unsigned long p   = now - last_pulse_us;
        last_pulse_us     = now;
        if (p > 200UL) {
            period_buf[period_idx] = p;
            period_idx = (period_idx + 1) % FILTER_SIZE;
            got_pulse  = true;
        }
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

        unsigned long sum   = 0;
        uint8_t       count = 0;
        for (uint8_t i = 0; i < FILTER_SIZE; i++) {
            if (buf[i] > 0) { sum += buf[i]; count++; }
        }
        if (count == 0) return 0.0f;

        return 60000000.0f / ((float)(sum / count) * 4.0f * PPR);
    }

    void setMotor(float pwm)
    {
        if (pwm >= 0.0f) {
            digitalWrite(pinIN1, HIGH);
            digitalWrite(pinIN2, LOW);
        } else {
            digitalWrite(pinIN1, LOW);
            digitalWrite(pinIN2, HIGH);
            pwm = -pwm;
        }
        analogWrite(pinPWM, (int)constrain(pwm, 0.0f, 255.0f));
    }

    void update()
    {
        rpm = measureRPM();
        float error = setpoint - rpm;

        integral += error * Ts;
        integral  = constrain(integral, -200.0f, 200.0f);

        float derivative = (error - last_error) / Ts;
        output = Kp * error + Ki * integral + Kd * derivative;
        output = constrain(output, 0.0f, 255.0f);
        last_error = error;

        setMotor(output);
    }

    void printSerial(const char* name)
    {
        Serial.print(name); Serial.print("_rpm:"); Serial.print(rpm, 2);
        Serial.print(",");
        Serial.print(name); Serial.print("_sp:");  Serial.print(setpoint);
        Serial.print(",");
        Serial.print(name); Serial.print("_out:"); Serial.print(output);
    }
};

//      IN1                   IN2                   PWM
//      ENC_A                 ENC_B
MotorPID m1(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0],
            Pins::kEncoders[1],    Pins::kEncoders[0]);   // UL  A=39 B=40

MotorPID m2(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1],
            Pins::kEncoders[2],    Pins::kEncoders[3]);   // UR  A=15 B=14

MotorPID m3(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2],
            Pins::kEncoders[4],    Pins::kEncoders[5]);   // LL  A=41 B=13

MotorPID m4(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3],
            Pins::kEncoders[6],    Pins::kEncoders[7]);   // LR  A=17 B=16

//  ISRs — one per motor fase
void isr_m1_A() { m1.onPulse(); }
void isr_m1_B() { m1.onPulse(); }
void isr_m2_A() { m2.onPulse(); }
void isr_m2_B() { m2.onPulse(); }
void isr_m3_A() { m3.onPulse(); }
void isr_m3_B() { m3.onPulse(); }
void isr_m4_A() { m4.onPulse(); }
void isr_m4_B() { m4.onPulse(); }

unsigned long last_time = 0;
const float   Ts        = 0.05f;

void setup()
{
    Serial.begin(115200);
    analogWriteResolution(8);

    m1.begin(); m2.begin(); m3.begin(); m4.begin();

    // Initial setpoints  — change it depending on needs
    m1.setpoint = 40.0f;
    m2.setpoint = 40.0f;
    m3.setpoint = 40.0f;

    // Attach interrupts - both fases, both flancos
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[1]), isr_m1_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[0]), isr_m1_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[2]), isr_m2_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[3]), isr_m2_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[4]), isr_m3_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[5]), isr_m3_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[6]), isr_m4_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Pins::kEncoders[7]), isr_m4_B, CHANGE);

    last_time = millis();
}

void loop()
{
    unsigned long now = millis();
    if (now - last_time >= (unsigned long)(Ts * 1000.0f))
    {
        last_time += (unsigned long)(Ts * 1000.0f);

        m1.update();
        m2.update();
        m3.update();
        m4.update();

        m1.printSerial("m1");
        Serial.print(",");
        m2.printSerial("m2");
        Serial.print(",");
        m3.printSerial("m3");
        Serial.print(",");
        m4.printSerial("m4");
        Serial.println();
    }
}