/*

* @file wheels.cpp
*
* @author Ximena Patricia García Magdaleno
*
* @version 0.1
*
* @date 2026-01-12
* Seconc layer for control of CONTROL (2)

  */

#include "kinematics.hpp"
#include "constants.h"

#include <math.h>

static constexpr float DEG2RAD = Constants::DriveConstants::kDEG2RAD;

static constexpr float M1_ANGLE = Constants::Kinematics::M1_ANGLE;  //M1
static constexpr float M2_ANGLE = Constants::Kinematics::M2_ANGLE;  //M2
static constexpr float M3_ANGLE = Constants::Kinematics::M3_ANGLE;  //M3
static constexpr float M4_ANGLE = Constants::Kinematics::M4_ANGLE;  //M4

void OmniMotors::normalize4(float &a, float &b, float &c, float &d)
{
// NORMALIZE wheels together
float m = fabsf(a);
m = fmaxf(m, fabsf(b));
m = fmaxf(m, fabsf(c));
m = fmaxf(m, fabsf(d));

if (m > 1.0f) { a/=m; b/=m; c/=m; d/=m; }
}

void OmniMotors::setWheelCmd(DCMotor& m, float cmd) {
// "cdm" how strongly and in which direction the wheel rotates <-1 || 1>
cmd = constrain(cmd, -1.0f, 1.0f);

auto dir = (cmd >= 0.0f) ? DCMotor::Direction::FORWARD
: DCMotor::Direction::BACKWARD;

int pwm = (int)(fabsf(cmd) * 255.0f);
m.move(pwm, dir); //m object, "pwm" and "dir" attributes>>> direction

}

void OmniMotors::MoveXYW(float vx, float vy, float omega)
{
// Limits omega so that it always has a real effect
// even when vx/vy are at their maximum
constexpr float kOmegaShare = 0.3f;
omega = constrain(omega, -kOmegaShare, +kOmegaShare);

float m1 = vx * cosf(M1_ANGLE * DEG2RAD) + vy * sinf(M1_ANGLE * DEG2RAD) + omega;
float m2 = vx * cosf(M2_ANGLE * DEG2RAD) + vy * sinf(M2_ANGLE * DEG2RAD) + omega;
float m3 = vx * cosf(M3_ANGLE * DEG2RAD) + vy * sinf(M3_ANGLE * DEG2RAD) + omega;
float m4 = vx * cosf(M4_ANGLE * DEG2RAD) + vy * sinf(M4_ANGLE * DEG2RAD) + omega;

normalize4(m1, m2, m3, m4);

setWheelCmd(upper_left_,  m1);
setWheelCmd(upper_right_, m2);
setWheelCmd(lower_left_,  m3);
setWheelCmd(lower_right_, m4);

}
