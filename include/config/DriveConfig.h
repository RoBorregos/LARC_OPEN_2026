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
} // namespace Constants
