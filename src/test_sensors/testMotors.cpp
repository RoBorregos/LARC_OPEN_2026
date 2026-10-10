/**
 * @file testMotors.cpp
 * @brief Standalone diagnostic (no RTOS): moves each chassis motor
 *        (M1-M4) forward then backward one by one via
 *        DCMotor::testForwardBackward(). No BNO/PID/EKF/state machine.
 *
 * pio run -e test_motors -t upload -t monitor
 */

#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include "motors.hpp"

static constexpr float diameter = Constants::DriveConstants::kWheelDiameter;

//Encoders are not used yet
// Motor 1 (UL):
static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];
// Motor 2 (UR):
static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];
// Motor 3 (LL):
static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
// Motor 4 (LR):
static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];



DCMotor m1_ul(34, 33, 8, true, UL_ENC_A, UL_ENC_B, diameter);  //M1
DCMotor m2_ur(37, 38, 10, true, LL_ENC_A, LL_ENC_B, diameter); //M3
DCMotor m3_ll(36, 35, 9, true, UR_ENC_A, UR_ENC_B, diameter); //M2
DCMotor m4_lr(40, 39, 11, true, LR_ENC_A, LR_ENC_B, diameter); //M4


void setup() {
  Serial.begin(115200);
  m1_ul.begin();
  m2_ur.begin();
  m3_ll.begin();
  m4_lr.begin();

  Serial.println("=== Motor test: forward/backward one-by-one ===");
}

void loop() {
  static constexpr int      pwm   = 120;
  static constexpr uint32_t time  = 3000;
  static constexpr uint32_t pause = 600;

  Serial.println("=== Motor test: all together, forward ===");
  m1_ul.move(pwm);
  m2_ur.move(pwm);
  m3_ll.move(pwm);
  m4_lr.move(pwm);
  delay(time);

  m1_ul.stop();
  m2_ur.stop();
  m3_ll.stop();
  m4_lr.stop();
  delay(pause);

  Serial.println("=== Motor test: all together, backward ===");
  m1_ul.move(-pwm);
  m2_ur.move(-pwm);
  m3_ll.move(-pwm);
  m4_lr.move(-pwm);
  delay(time);

  m1_ul.stop();
  m2_ur.stop();
  m3_ll.stop();
  m4_lr.stop();
  delay(pause);
}
