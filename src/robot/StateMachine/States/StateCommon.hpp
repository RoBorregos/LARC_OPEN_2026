/*
*@author:  Ximena Patricia García Magdaleno
* StateCommon.hpp
* Shared constants and QTR corrections for the States (based on DriveStateMachineTest)
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"

namespace StateCommon
{
    static constexpr float kVelocity = 0.30f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    static constexpr float kCornerSetpoint = 2900.0f;
    static constexpr float kCornerKp = 0.00012f;
    static constexpr float kCornerCorrMax = 0.20f;
    static constexpr float kCornerCorrAlpha = 0.15f;

    static constexpr int kQtrPosMax = (QTR::N - 1) * 1000; // 6000 (indice 6 * 1000)
    static constexpr float kRearCorrAlpha = 0.15f;

    static constexpr uint32_t kStartIgnoreTimeMs = 4500;    // Time to ignore IR's at the START point
    static constexpr uint32_t kClearDelayMs = 1500;         // Tiempo para cambiar nuevamente a Forward
    static constexpr uint32_t kNoObstacleToCornerMs = 1000; // Time without obstacle to go forward and LOOKFORLINE

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTooCloseAvoidLeftMm  = 120;
    static constexpr uint16_t kTooCloseAvoidRightMm = 100;

    static constexpr bool kLimitSwitchConnected = false;

    // qtrFront -> vx correction target (0 when the line is lost)
    inline float frontCornerCorrTarget(bool onLine)
    {
        if (!onLine)
            return 0.0f;
        const float error = kCornerSetpoint - qtrFront.getPosition();
        return constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }

    // qtrRear is wired mirrored to qtrFront
    inline float rearCornerCorrTarget()
    {
        if (!qtrRear.onLine())
            return 0.0f;
        const float mirroredRearPos = kQtrPosMax - qtrRear.getPosition();
        const float error = kCornerSetpoint - mirroredRearPos;
        return constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }
}
