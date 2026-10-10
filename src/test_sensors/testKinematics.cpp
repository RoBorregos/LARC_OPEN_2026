/**
 * @file testKinematics.cpp
 * @brief Standalone diagnostic (no RTOS): exercises OmniMotors::MoveXYW
 *        directly via Drive::testKinematics() (forward/backward/right/left/
 *        diagonals), with yaw-hold disabled -- to confirm the wheel mixing
 *        (M1-M4 kinematics) and pin/inversion mapping produce the physically
 *        expected motion, without BNO PID or EKF odometry in the loop.
 *
 * pio run -e test_kinematics -t upload -t monitor
 */

#include <Arduino.h>
#include "subsystem/Drive/Drive.hpp"

Drive drive;

void setup() {
  Serial.begin(115200);
  delay(500);

  drive.begin();
  Serial.println("=== Kinematics test: fwd/back/right/left/diag ===");
}

void loop() {
  drive.testKinematics(0.30f, 12000);
}
