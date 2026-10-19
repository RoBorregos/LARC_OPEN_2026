/**
 * @file DriveClosedLoopTest.cpp
 * @brief Full Drive stack (BNO085 yaw hold + omni kinematics + per-wheel RPM PID)
 *        cycling forward / backward / left / right with a stop between each.
 *        Prints setpoint, measured rpm and PWM of every wheel for the serial plotter.
 *
 * Set CLOSED_LOOP = false to compare against the old open-loop drive.
 *
 * pio run -e drive_closed_loop_test -t upload -t monitor
 */

#include <Arduino.h>
#include "subsystem/Drive/Drive.hpp"

Drive drive;

static constexpr bool     CLOSED_LOOP = true;
static constexpr float    kSpeed      = Constants::PID::kcurrentVelocity;
static constexpr uint32_t PHASE_MS    = 3000;
static constexpr uint32_t STOP_MS     = 1500;
static constexpr uint32_t PRINT_MS    = 100;

static const char* kNames[4] = {"UL", "UR", "LL", "LR"};

uint32_t t0        = 0;
uint32_t lastPrint = 0;

void applyPhase(uint8_t phase) {
    switch (phase) {
        case 0: drive.forward(kSpeed);  break;
        case 1: drive.backward(kSpeed); break;
        case 2: drive.left(kSpeed);     break;
        case 3: drive.right(kSpeed);    break;
    }
}

void setup() {
    drive.begin();
    drive.setClosedLoop(CLOSED_LOOP);
    drive.holdYaw(true);
    t0 = millis();
}

void loop() {
    drive.update();

    const uint32_t t     = (millis() - t0) % (4 * (PHASE_MS + STOP_MS));
    const uint8_t  phase = t / (PHASE_MS + STOP_MS);
    if (t % (PHASE_MS + STOP_MS) < PHASE_MS) applyPhase(phase);
    else                                    drive.stop();

    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    for (uint8_t i = 0; i < 4; i++) {
        Serial.print("sp_");  Serial.print(kNames[i]); Serial.print(':');
        Serial.print(drive.getWheelSetpoint(i), 1);   Serial.print(',');
        Serial.print("rpm_"); Serial.print(kNames[i]); Serial.print(':');
        Serial.print(drive.getWheelRpm(i), 1);        Serial.print(',');
        Serial.print("pwm_"); Serial.print(kNames[i]); Serial.print(':');
        Serial.print(drive.getWheelPwm(i));           Serial.print(',');
    }
    Serial.print("yawDeg:");
    Serial.println(drive.getYaw() * 180.0f / M_PI, 2);
}
