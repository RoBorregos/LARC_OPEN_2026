#ifndef OMNI_MOTORS_HPP
#define OMNI_MOTORS_HPP

#include <Arduino.h>
#include "motors.hpp"

class OmniMotors

{

public:
  OmniMotors(DCMotor& ul, DCMotor& ur, DCMotor& ll, DCMotor& lr)
    : upper_left_(ul), upper_right_(ur), lower_left_(ll), lower_right_(lr) {}

  void begin() {
    upper_left_.begin();
    upper_right_.begin();
    lower_left_.begin();
    lower_right_.begin();
  }

  void MoveXYW(float vx, float vy, float omega);

  // Wheel commands in [-1, 1] (UL, UR, LL, LR) without driving the motors
  void ComputeXYW(float vx, float vy, float omega, float out[4]);

  void Stop() {
    upper_left_.stop();
    upper_right_.stop();
    lower_left_.stop();
    lower_right_.stop();
  }

private:
  DCMotor& upper_left_;
  DCMotor& upper_right_;
  DCMotor& lower_left_;
  DCMotor& lower_right_;

  void setWheelCmd(DCMotor& m, float cmd);
  void normalize4(float &a, float &b, float &c, float &d);

};

#endif
