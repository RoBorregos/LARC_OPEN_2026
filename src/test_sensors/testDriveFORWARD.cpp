/**
 * @file testDriveFORWARD.cpp
 * @brief Standalone diagnostic (no RTOS): drives forward indefinitely
 *        through the full Drive class (BNO085 yaw-hold PID + omni kinematics
 *        + EKF odometry), to isolate the yaw-hold correction in a straight
 *        line without the turns/stops from the square in testDrive.cpp.
 *
 * pio run -e test_drive_forward -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pins.h"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive drive;


static uint32_t stepStart = 0;

static constexpr uint32_t kStepMs = 8000;
static constexpr float kSpeed = Constants::PID::kcurrentVelocity;

void setup() {
  Serial.begin(115200);
  delay(500);

  drive.begin();
  drive.holdYaw(true);
  drive.setTargetYaw(drive.getYaw());

}

void loop() {
  //Actualiza drive <<<BNO + PID + motores>>>
  drive.update();

  drive.forward(kSpeed);
}
