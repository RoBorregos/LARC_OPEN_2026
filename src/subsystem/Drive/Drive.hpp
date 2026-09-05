// Third layer for control of CONTROL (3)
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "constants.h"
#include "pins.h"
#include "BNO085/BNO085.hpp"
#include "motors.hpp"
#include "kinematics.hpp"
#include "PIDController.hpp"
#include "qtr.hpp"
#include "OdometryEKF.hpp"

static constexpr float diameter = Constants::DriveConstants::kWheelDiameter;

static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];

static constexpr uint8_t UR_ENC_A = Pins::kLimitSwitch;
static constexpr uint8_t UR_ENC_B = Pins::kLimitSwitch2;

static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];

class Drive {
public:
    Drive();

    void begin();
    void update();

    // Basic movement
    void forward(float speed);
    void backward(float speed);
    void left(float speed);
    void right(float speed);
    void diagonalLeft(float speed);
    void stop();
    void brake();
    void setTranslation(float vx, float vy);

    // autonomous movement
    void driveDistance(float meters, float speed);
    void strafeDistance(float meters, float speed);
    void turnTo(float targetDeg, uint32_t timeoutMs = 3000);
    bool isBusy() const { return busy_; }

    // Yaw hold 
    void  holdYaw(bool enable);
    void  setTargetYaw(float yawRad);
    float getYaw() const;

    //  Omega manual 
    void setManualOmega(float omegaRadS);
    void clearManualOmega();

    //  QTR 
    void attachQTR(QTR& qtr);
    void followLine(float vySpeed);
    void stopLineFollow();
    bool isFollowingLine() const { return lineFollowEnabled_; }

    //  Odometry 
    void  resetOdometry();
    float getOdoX() const { return ekf_.getX(); }
    float getOdoY() const { return ekf_.getY(); }

    //  Test / debug 
    void testKinematics(float v = 0.45f, uint32_t T = 2000);

    float getVxCmd() const { return vxCmd_; }
    float getVyCmd() const { return vyCmd_; }

    float getM1Meters() const { return const_cast<Drive*>(this)->m1_ul_.getDistanceMeters(); }
    float getM2Meters() const { return const_cast<Drive*>(this)->m2_ur_.getDistanceMeters(); }
    float getM3Meters() const { return const_cast<Drive*>(this)->m3_ll_.getDistanceMeters(); }
    float getM4Meters() const { return const_cast<Drive*>(this)->m4_lr_.getDistanceMeters(); }

    void beginNoBNO();
    void updateNoBNO();

    void moveMotorUL(int pwm) { m1_ul_.move(pwm); }
    void moveMotorUR(int pwm) { m2_ur_.move(pwm); }
    void moveMotorLL(int pwm) { m3_ll_.move(pwm); }
    void moveMotorLR(int pwm) { m4_lr_.move(pwm); }

    void setRPMs(float ul, float ur, float ll, float lr);

private:
    static float rad2deg(float r);
    static float deg2rad(float d);
    static float clampf(float x, float lo, float hi);
    static float wrapAngle(float a);

    void updateOdometry();
    void updateBusy(uint32_t now);

    // Hardware
    BNO085     bno_;
    DCMotor    m1_ul_, m2_ur_, m3_ll_, m4_lr_;
    OmniMotors omni_;

    // Yaw PID 
    static constexpr float P         = Constants::PID::kKp;
    static constexpr float I         = Constants::PID::kKi;
    static constexpr float D         = Constants::PID::kKd;
    static constexpr float kOmegaMax = Constants::PID::kOmegaMax;

    PIDController yawPid_;
    float targetYaw_          = 0.0f;
    bool  yawHoldEnabled_     = true;
    bool  manualOmegaEnabled_ = false;
    float manualOmega_        = 0.0f;

    // Velocity command
    float vxCmd_ = 0.0f;
    float vyCmd_ = 0.0f;

    // Movimiento autónomo 
    bool     busy_        = false;
    float    distTarget_  = 0.0f;
    float    busyVx_      = 0.0f;
    float    busyVy_      = 0.0f;
    uint32_t busyStart_   = 0;
    uint32_t busyTimeout_ = 5000;

    // EKF Odometry 
    OdometryEKF ekf_;
    float prevD1_ = 0.0f;
    float prevD2_ = 0.0f;
    float prevD3_ = 0.0f;
    float prevD4_ = 0.0f;

    //  QTR / line follow 
    QTR*          qtrFront_          = nullptr;
    PIDController linePid_;
    bool          lineFollowEnabled_ = false;
    float         lineSpeedVy_       = 0.0f;

    static constexpr float kLineCenter   = 3500.0f;
    static constexpr float kLinePidMax   = 0.20f;
    static constexpr float kLineDeadband = 150.0f;

    //  Timing 
    static constexpr uint32_t kControlMs = 10;
    static constexpr uint32_t kPrintMs   = 100;
    uint32_t lastControl_ = 0;
    uint32_t lastPrint_   = 0;

    float rpmSetUL_ = 0.0f;
    float rpmSetUR_ = 0.0f;
    float rpmSetLL_ = 0.0f;
    float rpmSetLR_ = 0.0f;
    bool  rpmModeEnabled_ = false;
};