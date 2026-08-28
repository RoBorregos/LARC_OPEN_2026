/**
 * @file sensors_motors_test.cpp
 * @brief Standalone diagnostic (no RTOS, no state machine, no PID/EKF):
 *        continuously prints ToF (VLX), IR line sensors, the front QTR and
 *        the BNO085 yaw while cycling the chassis through
 *        forward/backward/left/right, so sensor readings and motor movement
 *        can be checked together in one run. Same wiring/pins as production
 *        (see instances.cpp/Drive.cpp). BNO085 on Wire2, see BNO085::begin().
 *
 * pio run -e sensors_motors_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"

#include "motors.hpp"
#include "kinematics.hpp"

#include "IR/IR.hpp"
#include "mux.h"
#include "qtr.hpp"
#include "TCA9548A/TCA9548A.h"
#include "tof/tof.hpp"
#include "BNO085/BNO085.hpp"

// ---- Motors: same wiring/inversion as Drive.cpp / motor_test.cpp (chassis only, no M5) ----
DCMotor m1_ul(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], true, Pins::kEncoders[1], Pins::kEncoders[0]);

// M2's encoder pins (Pins::kEncoders[2]/[3] = 25/24) are the Teensy 4.1's
// fixed Wire2 pins (SDA2/SCL2), used here by the BNO085. DCMotor::begin()
// unconditionally does pinMode()+attachInterrupt() on its encoder pins
// (see motors.cpp), which steals them from the I2C peripheral even with no
// physical encoder wired -- there's no encoder on M2 right now anyway, so
// point it at unused pins instead (same placeholder trick motor_test.cpp
// uses for M5's missing encoder) to keep Wire2 free for the BNO085.
DCMotor m2_ur(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true, Pins::kLimitSwitch, Pins::kLimitSwitch2);

DCMotor m3_ll(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true, Pins::kEncoders[4], Pins::kEncoders[5]);
DCMotor m4_lr(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], true, Pins::kEncoders[6], Pins::kEncoders[7]);
OmniMotors omni(m1_ul, m2_ur, m3_ll, m4_lr);

// ---- Sensors: same wiring as robot/instances/instances.cpp ----
Mux74HC4067 mux(Pins::kMuxSig);
QTR qtrFront(Pins::kQtrFrontFirstCh, mux);

const uint8_t irPins[IRLine::N] = {
    Pins::kIrChFL, Pins::kIrChFR, Pins::kIrChBL, Pins::kIrChBR
};
IRLine ir(irPins, 0b0000);

TCA9548A i2cMux;
ToF tofLeft(Pins::kToFchFL, i2cMux, ToFType::L1X);
ToF tofRight(Pins::kToFchFR, i2cMux, ToFType::L1X);

BNO085 bno; // Wire2, yaw only for this diagnostic

constexpr uint32_t kPrintPeriodMs = 150;
constexpr float    kMoveSpeed     = 0.45f; // normalized -1..1, same default as Drive::testKinematics
constexpr uint32_t kPhaseMs       = 1500;
constexpr uint32_t kPauseMs       = 500;

enum class MovePhase { FORWARD, PAUSE1, BACKWARD, PAUSE2, LEFT, PAUSE3, RIGHT, PAUSE4 };

MovePhase movePhase   = MovePhase::FORWARD;
uint32_t  phaseStartMs = 0;

bool isPause(MovePhase phase)
{
    return phase == MovePhase::PAUSE1 || phase == MovePhase::PAUSE2 ||
           phase == MovePhase::PAUSE3 || phase == MovePhase::PAUSE4;
}

void applyPhase(MovePhase phase)
{
    switch (phase)
    {
        case MovePhase::FORWARD:  Serial.println(">> FORWARD");  omni.MoveXYW(+kMoveSpeed, 0.0f, 0.0f); break;
        case MovePhase::BACKWARD: Serial.println(">> BACKWARD"); omni.MoveXYW(-kMoveSpeed, 0.0f, 0.0f); break;
        case MovePhase::LEFT:     Serial.println(">> LEFT");     omni.MoveXYW(0.0f, -kMoveSpeed, 0.0f); break;
        case MovePhase::RIGHT:    Serial.println(">> RIGHT");    omni.MoveXYW(0.0f, +kMoveSpeed, 0.0f); break;
        default:                  omni.Stop(); break; // PAUSE*
    }
}

void advancePhase(uint32_t now)
{
    const uint32_t duration = isPause(movePhase) ? kPauseMs : kPhaseMs;
    if ((now - phaseStartMs) < duration)
        return;

    switch (movePhase)
    {
        case MovePhase::FORWARD:  movePhase = MovePhase::PAUSE1;   break;
        case MovePhase::PAUSE1:   movePhase = MovePhase::BACKWARD; break;
        case MovePhase::BACKWARD: movePhase = MovePhase::PAUSE2;   break;
        case MovePhase::PAUSE2:   movePhase = MovePhase::LEFT;     break;
        case MovePhase::LEFT:     movePhase = MovePhase::PAUSE3;   break;
        case MovePhase::PAUSE3:   movePhase = MovePhase::RIGHT;    break;
        case MovePhase::RIGHT:    movePhase = MovePhase::PAUSE4;   break;
        case MovePhase::PAUSE4:   movePhase = MovePhase::FORWARD;  break;
    }

    phaseStartMs = now;
    applyPhase(movePhase);
}

void printSensors()
{
    tofLeft.update();
    tofRight.update();
    ir.update();
    qtrFront.update();
    bno.update();

    Serial.print(F("VLX L:")); Serial.print(tofLeft.isValid()  ? tofLeft.getDistanceCm()  : -1.0f, 0);
    Serial.print(F("cm R:"));  Serial.print(tofRight.isValid() ? tofRight.getDistanceCm() : -1.0f, 0);

    Serial.print(F("cm | IR FL:")); Serial.print(ir.getState(IRLine::FL));
    Serial.print(F(" FR:"));        Serial.print(ir.getState(IRLine::FR));
    Serial.print(F(" BL:"));        Serial.print(ir.getState(IRLine::BL));
    Serial.print(F(" BR:"));        Serial.print(ir.getState(IRLine::BR));

    Serial.print(F(" | QTR pos:")); Serial.print(qtrFront.getPosition());
    Serial.print(F(" onLine:"));    Serial.print(qtrFront.onLine());

    Serial.print(F(" | Yaw:")); Serial.print(bno.getYaw() * (180.0f / PI), 1);
    Serial.print(F("deg"));

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("  SENSORS + MOTORS TEST (VLX/IR/QTR + chassis)");
    Serial.println("================================");

    // ToF (VLX) via TCA9548A -- same sequence as StateMachine::begin()
    Wire.begin();
    Wire.setClock(400000);
    i2cMux.begin();

    const bool okL = tofLeft.begin();
    const bool okR = tofRight.begin();
    Serial.print("tofLeft init: ");  Serial.println(okL ? "OK" : "FAIL");
    Serial.print("tofRight init: "); Serial.println(okR ? "OK" : "FAIL");

    tofLeft.setMaxRange(600);
    tofRight.setMaxRange(600);
    tofLeft.setUpdateInterval(30);
    tofRight.setUpdateInterval(30);

    // IR + front QTR (QTR::begin() also does mux.begin() internally)
    ir.begin();
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);

    // BNO085 (Wire2) -- blocks retrying 0x4A/0x4B until it connects, see BNO085::begin()
    bno.begin();

    // Chassis motors
    analogWriteResolution(8);
    omni.begin();

    Serial.println("Listo. Moviendo el chasis en ciclo mientras se leen los sensores...");
    Serial.println();

    phaseStartMs = millis();
    applyPhase(movePhase);
}

void loop()
{
    const uint32_t now = millis();

    advancePhase(now);

    static uint32_t lastPrintMs = 0;
    if ((now - lastPrintMs) >= kPrintPeriodMs)
    {
        lastPrintMs = now;
        printSensors();
    }
}
