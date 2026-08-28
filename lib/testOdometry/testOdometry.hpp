#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pins.h"
#include "BNO085/BNO085.hpp"

class OdomMovement
{
public:
    OdomMovement();

    void begin();
    void update();
    void setCommandTimeout(uint32_t timeoutMs);

    void forward(float rpm);
    void backward(float rpm);
    void right(float rpm);
    void left(float rpm);

    void stop();
    void resetPose();

    float getX() const;
    float getY() const;
    float getTheta() const;
    float getThetaDeg() const;
    float getDistance() const;

    float getForwardProgress() const;
    float getLateralProgress() const;

    float getYawNow() const;
    void captureCurrentYawTarget();

    //Odometry
    float getRpmUL() const { return lastRpmUL_; }
    float getRpmUR() const { return lastRpmUR_; }
    float getRpmLL() const { return lastRpmLL_; }
    float getRpmLR() const { return lastRpmLR_; }

    // qtr correction
    void setTranslation(float vx_rpm, float vy_rpm);

    void setRPMs(float ul, float ur, float ll, float lr);

private:
    static constexpr uint8_t FILTER_SIZE = 8;

    static constexpr float kTs          = 0.05f;
    static constexpr float kWheelRadius = 0.054f;
    static constexpr float kInvSqrt2    = 0.70710678f;
    static constexpr float kOdomScale   = 2.57f;
    static constexpr float kPwmDeadband = 60.0f;
    static constexpr float kPwmMax      = 150.0f;

    static constexpr float kKp = 1.5f;
    static constexpr float kKi = 1.2f;
    static constexpr float kKd = 0.005f;

    static constexpr float kYawKp  = 150.0f;
    static constexpr float kYawKi  = 0.5f;
    static constexpr float kYawKd  = 0.5f;
    static constexpr float kYawMax = 30.0f;

    struct Motor
    {
        uint8_t pwmPin, in1, in2;
        volatile unsigned long period_buf[FILTER_SIZE];
        volatile uint8_t period_idx;
        volatile unsigned long last_pulse_us;
        volatile bool got_pulse;
        float PPR;
        float Kp, Ki, Kd;
        float setpoint;
        float integral;
        float last_error;
        bool inverted;

        
    };

    static OdomMovement* instance_;

    BNO085 bno_;

    uint8_t encUL_A_, encUL_B_, pwmUL_, inUL1_, inUL2_;
    uint8_t encUR_A_, encUR_B_, pwmUR_, inUR1_, inUR2_;
    uint8_t encLL_A_, encLL_B_, pwmLL_, inLL1_, inLL2_;
    uint8_t encLR_A_, encLR_B_, pwmLR_, inLR1_, inLR2_;

    Motor UL_, UR_, LL_, LR_;

    volatile long ticksLL_count_;
    long prevTicksLL_;

    float yawTarget_, yawIntegral_, yawPrevErr_, yawNow_;

    float ekf_x_, ekf_y_, ekf_th_;
    float P_[3][3];
    float Q_[3][3];
    float R_theta_;

    uint32_t lastCycleMs_;
    uint32_t lastCommandMs_ = 0;
    uint32_t commandTimeoutMs_ = 100;
    bool commandEnabled_ = false;

    static float wrapPi(float a);
    static void pushPeriod(Motor& m, unsigned long p);

    float yawPidStep(float yawMeasured, float dt);

    float measureRPM(Motor& m);
    float measureRPM_LL(float dtSec);

    void setMotorPWM(Motor& m, float pwm);
    void stopMotor(Motor& m);
    void stopAll();
    void markCommandReceived();

    void pidStepWithRPM(Motor& m, float rpm, float extraRPM);
    void ekfStep(float dt, float rpmUL, float rpmUR, float rpmLL, float rpmLR);


    static void isrUL_A();
    static void isrUL_B();
    static void isrUR_A();
    static void isrUR_B();
    static void isrLR_A();
    static void isrLR_B();
    static void isrLL();

    //Odometry
    float lastRpmUL_ = 0.0f;
    float lastRpmUR_ = 0.0f;
    float lastRpmLL_ = 0.0f;
    float lastRpmLR_ = 0.0f;

};
