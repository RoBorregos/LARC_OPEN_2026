/**
 * @file DriveConfig.h
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Drive constants: kinematics, wheel geometry and yaw PID.
 */

#pragma once

#include <Arduino.h>
#include <math.h>

namespace Constants
{
    namespace Kinematics
    {
        // "omni_motors" class
        constexpr float M1_ANGLE = 135.0f;  // M1     UL
        constexpr float M2_ANGLE = 45.0f;   // M2     UR
        constexpr float M3_ANGLE = -135.0f; // M3     LL
        constexpr float M4_ANGLE = -45.0f;  // M4     LR
    } // namespace Kinematics

    namespace DriveConstants
    {
        static constexpr float kDEG2RAD = PI / 180.0f;

        constexpr float kWheelDiameter = 0.109f;
        constexpr float kWheelRaius = kWheelDiameter / 2.0;
        constexpr float kWheelCircumference = 2 * M_PI * kWheelRaius;
    } // namespace DriveConstants

    // Yaw-hold PID
    namespace PID
    {
        static constexpr float kKp = 1.20f;//1.20;   // 4.8f;  2.5f; 1.5f
        static constexpr float kKi = 0.002f; // 0.002f
        static constexpr float kKd = 0.04f;  // 0.0042f; 0.0012f; 0.06f; 0.0f
        static constexpr float kOmegaMax = 0.25f; //0.25f;
        static constexpr float kOmegaMin = 0.12f;       // minimum omega that moves the motors (deadband)
        static constexpr float kYawDeadbandDeg = 1.5f;  // below this yaw error no minimum push
        static constexpr float kYawRateClosingDegS = 5.0f; // turning toward the target faster than this: no minimum push
        static constexpr float kcurrentVelocity = 0.30f; // Velocity according to PID
    } // namespace PID

    // Per-wheel RPM loop. Values copied from the sketches marked "Calibrated A7 440: check"
    // in src/test_sensors/byMotor/{UL,UR,LL,LR}EncoderCalibration.cpp.
    // RPM is in calibration units: quadrature edges / (4 * ppr).
    namespace WheelPID
    {
        struct WheelGains
        {
            float kp, ki, kd;
            float ffOffset, ffSlope; // PWM = ffOffset + ffSlope * rpm
            float ppr;
            float iMaxPwm;           // cap on Ki * integral
        };

        //                         kp    ki     kd    ffOff  ffSlope  ppr     iMax
        constexpr WheelGains kUL = {0.5f, 0.6f,  0.0f, 32.0f, 1.05f,  190.0f, 255.0f};
        constexpr WheelGains kUR = {0.5f, 0.6f,  0.0f, 30.0f, 1.05f,  188.0f, 255.0f};
        constexpr WheelGains kLL = {0.5f, 0.18f, 0.0f,  8.0f, 1.85f,  188.0f,  40.0f};
        constexpr WheelGains kLR = {0.5f, 0.8f,  0.0f, 32.0f, 1.30f,  189.0f, 255.0f};

        constexpr uint32_t kTsMs     = 50;     // same Ts as the calibration
        constexpr float    kRpmAlpha = 0.2f;   // EMA on measured rpm
        constexpr float    kSpRamp   = 90.0f;  // rpm/s
        // 0 = Teensy default. 25000 (used in the calibration sketches) left the robot unable to drive backward/sideways under Drive.
        constexpr uint32_t kPwmFreq  = 0;

        // Wheel command 1.0 from the kinematics maps to this rpm.
        // 120 keeps kcurrentVelocity (0.30) at ~25 rpm per wheel, close to the old open-loop speed.
        constexpr float kMaxWheelRpm = 120.0f;
    } // namespace WheelPID
} // namespace Constants
