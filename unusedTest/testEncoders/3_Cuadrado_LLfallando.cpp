#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pins.h"
#include "BNO/bno.hpp"

BNO bno1;

// Configuration
static constexpr float kSideMeters  = 0.80f;
static constexpr float kTargetRPM   = 48.0f;
static constexpr float kStopTolM    = 0.03f;
static constexpr float kTs          = 0.05f;
static constexpr float kWheelRadius = 0.054f;
static constexpr float kInvSqrt2    = 0.70710678f;
static constexpr float kOdomScale   = 2.57f;
static constexpr uint32_t kPausems  = 600;
static constexpr float kPwmDeadband = 48.0f;
static constexpr float kPwmMax      = 100.0f;

static constexpr float kKp = 1.5f;
static constexpr float kKi = 1.2f;
static constexpr float kKd = 0.005f;

#define FILTER_SIZE 8

// Pins
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

// LL uses pins from the code that worked:
// A = kEncoders[4] = pin 41
// B = kEncoders[5] = pin 13
// ISR on RISING on channel A, reads channel B for direction
const uint8_t encLL_A = Pins::kEncoders[4];  // pin 41
const uint8_t encLL_B = Pins::kEncoders[5];  // pin 13
const uint8_t pwmLL   = Pins::kPwmPin[2];    // pin 29
const uint8_t inLL1   = Pins::kLowerMotors[0];
const uint8_t inLL2   = Pins::kLowerMotors[1];

const uint8_t encLR_A = Pins::kEncoders[6];
const uint8_t encLR_B = Pins::kEncoders[7];
const uint8_t pwmLR   = Pins::kPwmPin[3];
const uint8_t inLR1   = Pins::kLowerMotors[2];
const uint8_t inLR2   = Pins::kLowerMotors[3];

// Motor struct
struct Motor {
    uint8_t pwmPin, in1, in2;
    volatile unsigned long period_buf[FILTER_SIZE];
    volatile uint8_t       period_idx;
    volatile unsigned long last_pulse_us;
    volatile bool          got_pulse;
    float PPR;
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    bool  inverted;
};

//                                                                            inverted
Motor UL = { pwmUL, inUL1, inUL2, {}, 0, 0, false, 482.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, false };
Motor UR = { pwmUR, inUR1, inUR2, {}, 0, 0, false, 475.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, true  };
Motor LL = { pwmLL, inLL1, inLL2, {}, 0, 0, false, 495.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, false };
Motor LR = { pwmLR, inLR1, inLR2, {}, 0, 0, false, 486.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, true  };

// ISR UL, UR, LR - period method
inline void pushPeriod(Motor &m, unsigned long p) {
    m.period_buf[m.period_idx] = p;
    m.period_idx = (m.period_idx + 1) % FILTER_SIZE;
    m.got_pulse  = true;
}

void isrUL_A() { unsigned long n=micros(); unsigned long p=n-UL.last_pulse_us; UL.last_pulse_us=n; if(p>200) pushPeriod(UL,p); }
void isrUL_B() { unsigned long n=micros(); unsigned long p=n-UL.last_pulse_us; UL.last_pulse_us=n; if(p>200) pushPeriod(UL,p); }
void isrUR_A() { unsigned long n=micros(); unsigned long p=n-UR.last_pulse_us; UR.last_pulse_us=n; if(p>200) pushPeriod(UR,p); }
void isrUR_B() { unsigned long n=micros(); unsigned long p=n-UR.last_pulse_us; UR.last_pulse_us=n; if(p>200) pushPeriod(UR,p); }
void isrLR_A() { unsigned long n=micros(); unsigned long p=n-LR.last_pulse_us; LR.last_pulse_us=n; if(p>200) pushPeriod(LR,p); }
void isrLR_B() { unsigned long n=micros(); unsigned long p=n-LR.last_pulse_us; LR.last_pulse_us=n; if(p>200) pushPeriod(LR,p); }

// ISR LL - RISING method + channel B for direction
// Same as the code that worked: ISR on RISING on channel A,
// reads channel B to determine direction (HIGH=forward, LOW=backward)
volatile long ticksLL_count = 0;

void isrLL() {
    if (digitalRead(encLL_B) == HIGH)
        ticksLL_count++;
    else
        ticksLL_count--;
}

long prevTicksLL = 0;

float measureRPM_LL(float dtSec) {
    noInterrupts();
    long cur = ticksLL_count;
    interrupts();

    long dTicks = cur - prevTicksLL;
    prevTicksLL = cur;  // advances only here - don't call twice per cycle

    // PPR without *4 because we only count RISING on channel A (1 edge per pulse)
    float mag = ((float)abs(dTicks) / 495.0f) / dtSec * 60.0f;
    return (LL.setpoint >= 0.0f) ? mag : -mag;
}

// RPM for UL, UR, LR - period method
float measureRPM(Motor &m) {
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t i=0; i<FILTER_SIZE; i++) buf[i] = m.period_buf[i];
    unsigned long last = m.last_pulse_us;
    bool has = m.got_pulse;
    interrupts();

    if (!has) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    unsigned long sum=0; uint8_t cnt=0;
    for (uint8_t i=0; i<FILTER_SIZE; i++) {
        if (buf[i]>0) { sum+=buf[i]; cnt++; }
    }
    if (cnt==0) return 0.0f;

    float avg = (float)(sum/cnt);
    float mag = 60000000.0f / (avg * 4.0f * m.PPR);
    return (m.setpoint >= 0.0f) ? mag : -mag;
}

// Motor driver with deadband and inversion
void setMotorPWM(Motor &m, float pwm) {
    if (pwm == 0.0f) {
        analogWrite(m.pwmPin, 0);
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        return;
    }

    float absPwm = fabsf(pwm);
    absPwm = kPwmDeadband + (absPwm / 255.0f) * (kPwmMax - kPwmDeadband);
    absPwm = constrain(absPwm, kPwmDeadband, kPwmMax);

    bool goForward = (pwm > 0.0f);
    if (m.inverted) goForward = !goForward;

    if (goForward) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
    }
    analogWrite(m.pwmPin, (int)absPwm);
}

void stopMotor(Motor &m) {
    analogWrite(m.pwmPin, 0);
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    m.integral   = 0.0f;
    m.last_error = 0.0f;
    m.got_pulse  = false;
}

void stopAll() {
    stopMotor(UL); stopMotor(UR);
    stopMotor(LL); stopMotor(LR);
}

// PID
void pidStepWithRPM(Motor &m, float rpm) {
    if (m.setpoint == 0.0f) { stopMotor(m); return; }

    float error  = m.setpoint - rpm;
    m.integral  += error * kTs;
    m.integral   = constrain(m.integral, -150.0f, 150.0f);

    float deriv  = (error - m.last_error) / kTs;
    float output = m.Kp*error + m.Ki*m.integral + m.Kd*deriv;
    m.last_error = error;

    if (m.setpoint < 0.0f) output = -fabsf(output);
    else                   output =  fabsf(output);

    output = constrain(output, -255.0f, 255.0f);
    setMotorPWM(m, output);
}

// EKF
float ekf_x=0, ekf_y=0, ekf_th=0;
float P[3][3] = {{0.01f,0,0},{0,0.01f,0},{0,0,0.05f}};
float Q[3][3] = {{0.0005f,0,0},{0,0.0005f,0},{0,0,0.003f}};
float R_theta = 0.02f;

float wrapPi(float a) {
    while(a >  PI) a -= 2*PI;
    while(a < -PI) a += 2*PI;
    return a;
}

void ekfStep(float dt, float rpmUL, float rpmUR, float rpmLL, float rpmLR) {
    float w_UL = rpmUL * 2*PI/60.0f;
    float w_UR = rpmUR * 2*PI/60.0f;
    float w_LL = rpmLL * 2*PI/60.0f;
    float w_LR = rpmLR * 2*PI/60.0f;

    const float k = kInvSqrt2 * kWheelRadius / 4.0f;
    float vx = ( w_UL + w_UR + w_LL + w_LR) * k * kOdomScale;
    float vy = (-w_UL + w_UR - w_LL + w_LR) * k * kOdomScale;

    bno1.update();
    float z_th = bno1.getYaw();

    float c=cosf(ekf_th), s=sinf(ekf_th);
    float xp = ekf_x + (vx*c - vy*s)*dt;
    float yp = ekf_y + (vx*s + vy*c)*dt;
    float tp  = ekf_th;

    float F[3][3] = {{1,0,(-vx*s-vy*c)*dt},{0,1,(vx*c-vy*s)*dt},{0,0,1}};
    float FP[3][3]={0}, Pp[3][3]={0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k2=0;k2<3;k2++) FP[i][j]+=F[i][k2]*P[k2][j];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k2=0;k2<3;k2++) Pp[i][j]+=FP[i][k2]*F[j][k2];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Pp[i][j]+=Q[i][j];

    float yt=wrapPi(z_th-tp), Sc=Pp[2][2]+R_theta;
    float K[3]={Pp[0][2]/Sc, Pp[1][2]/Sc, Pp[2][2]/Sc};
    ekf_x=xp+K[0]*yt; ekf_y=yp+K[1]*yt; ekf_th=wrapPi(tp+K[2]*yt);

    float Pn[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) Pn[i][j]=Pp[i][j];
    for(int i=0;i<3;i++) { Pn[i][0]-=K[i]*Pp[2][0]; Pn[i][1]-=K[i]*Pp[2][1]; Pn[i][2]-=K[i]*Pp[2][2]; }
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) P[i][j]=Pn[i][j];
}

// Commands
//            UL     UR     LL     LR
// Forward:  +RPM   +RPM   +RPM   +RPM
// Backward: -RPM   -RPM   -RPM   -RPM
// Right:    +RPM   -RPM   -RPM   +RPM
// Left:     -RPM   +RPM   +RPM   -RPM

void setRPMs(float ul, float ur, float ll, float lr) {
    UL.setpoint=ul; UR.setpoint=ur;
    LL.setpoint=ll; LR.setpoint=lr;
    UL.integral=UR.integral=LL.integral=LR.integral=0;
    UL.last_error=UR.last_error=LL.last_error=LR.last_error=0;
    UL.got_pulse=UR.got_pulse=LL.got_pulse=LR.got_pulse=false;
}

void resetPose() { ekf_x=0; ekf_y=0; }

// Square sequence
enum class Step : uint8_t { FORWARD, RIGHT, BACKWARD, LEFT, PAUSE, DONE };
Step     step       = Step::FORWARD;
uint32_t pauseStart = 0;
uint8_t  stepCount  = 0;

void startStep(Step s) {
    step = s;
    resetPose();
    switch(s) {
        case Step::FORWARD:
            Serial.println(">>> FORWARD");
            setRPMs(+kTargetRPM, +kTargetRPM, +kTargetRPM, +kTargetRPM);
            break;
        case Step::RIGHT:
            Serial.println(">>> RIGHT");
            setRPMs(+kTargetRPM, -kTargetRPM, -kTargetRPM, +kTargetRPM);
            break;
        case Step::BACKWARD:
            Serial.println(">>> BACKWARD");
            setRPMs(-kTargetRPM, -kTargetRPM, -kTargetRPM, -kTargetRPM);
            break;
        case Step::LEFT:
            Serial.println(">>> LEFT");
            setRPMs(-kTargetRPM, +kTargetRPM, +kTargetRPM, -kTargetRPM);
            break;
        default: break;
    }
}

// Setup
void setup() {
    Serial.begin(115200);
    Wire.begin();
    bno1.begin();
    delay(300);

    uint8_t encPins[] = {encUL_A,encUL_B,encUR_A,encUR_B,encLL_A,encLL_B,encLR_A,encLR_B};
    for(uint8_t p : encPins) pinMode(p, INPUT_PULLUP);

    uint8_t motPins[] = {pwmUL,inUL1,inUL2,pwmUR,inUR1,inUR2,pwmLL,inLL1,inLL2,pwmLR,inLR1,inLR2};
    for(uint8_t p : motPins) pinMode(p, OUTPUT);

    analogWriteResolution(8);
    analogWriteFrequency(pwmUL, 25000);
    analogWriteFrequency(pwmUR, 25000);
    analogWriteFrequency(pwmLL, 4000);   // pin 29 doesn't support 25kHz
    analogWriteFrequency(pwmLR, 25000);

    attachInterrupt(digitalPinToInterrupt(encUL_A), isrUL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUL_B), isrUL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUR_A), isrUR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUR_B), isrUR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLL_A), isrLL,   RISING); // <-- RISING, same as the code that worked
    attachInterrupt(digitalPinToInterrupt(encLR_A), isrLR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLR_B), isrLR_B, CHANGE);

    bno1.update();
    ekf_th = bno1.getYaw();

    Serial.println("=== CUADRADO START ===");
    delay(2000);
    startStep(Step::FORWARD);
}

// Loop
void loop() {
    static uint32_t lastCycle = millis();
    static uint32_t lastPrint = millis();
    uint32_t now = millis();

    if (now - lastCycle >= (uint32_t)(kTs*1000)) {
        float dt  = (now - lastCycle) / 1000.0f;
        lastCycle = now;

        // Medir RPM una sola vez por ciclo
        float rpmUL = measureRPM(UL);
        float rpmUR = measureRPM(UR);
        float rpmLL = measureRPM_LL(dt);  // prevTicksLL advances only here
        float rpmLR = measureRPM(LR);

        ekfStep(dt, rpmUL, rpmUR, rpmLL, rpmLR);

        pidStepWithRPM(UL, rpmUL);
        pidStepWithRPM(UR, rpmUR);
        pidStepWithRPM(LL, rpmLL);
        pidStepWithRPM(LR, rpmLR);

        if (now - lastPrint >= 200) {
            lastPrint = now;
            float dist = sqrtf(ekf_x*ekf_x + ekf_y*ekf_y);
            Serial.print("x:");     Serial.print(ekf_x, 3);
            Serial.print(" y:");    Serial.print(ekf_y, 3);
            Serial.print(" dist:"); Serial.print(dist, 3);
            Serial.print(" th:");   Serial.print(ekf_th*180/PI, 1);
            Serial.print(" | rpm UL:"); Serial.print(rpmUL, 0);
            Serial.print(" UR:");       Serial.print(rpmUR, 0);
            Serial.print(" LL:");       Serial.print(rpmLL, 0);
            Serial.print(" LR:");       Serial.print(rpmLR, 0);
            Serial.print(" | tLL:");    Serial.print(ticksLL_count);  // debug raw ticks
            Serial.print(" | step:");
            switch(step) {
                case Step::FORWARD:  Serial.print("FWD");  break;
                case Step::RIGHT:    Serial.print("RGT");  break;
                case Step::BACKWARD: Serial.print("BWD");  break;
                case Step::LEFT:     Serial.print("LFT");  break;
                case Step::PAUSE:    Serial.print("PAU");  break;
                case Step::DONE:     Serial.print("DONE"); break;
            }
            Serial.println();
        }
    }

    switch(step) {
        case Step::FORWARD:
        case Step::RIGHT:
        case Step::BACKWARD:
        case Step::LEFT: {
            float dist = sqrtf(ekf_x*ekf_x + ekf_y*ekf_y);
            if (dist >= kSideMeters - kStopTolM) {
                stopAll();
                Serial.print(">>> Llegue! dist=");
                Serial.println(dist, 3);
                pauseStart = now;
                step = Step::PAUSE;
            }
            break;
        }

        case Step::PAUSE: {
            if (now - pauseStart >= kPausems) {
                stepCount++;
                switch(stepCount) {
                    case 1: startStep(Step::RIGHT);    break;
                    case 2: startStep(Step::BACKWARD); break;
                    case 3: startStep(Step::LEFT);     break;
                    default:
                        step = Step::DONE;
                        Serial.println("=== CUADRADO COMPLETO ===");
                        break;
                }
            }
            break;
        }

        case Step::DONE:
            break;
    }
}