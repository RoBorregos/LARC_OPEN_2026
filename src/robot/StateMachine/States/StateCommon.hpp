/*
*@author:  Ximena Patricia García Magdaleno
* StateCommon.hpp
* Shared QTR corrections for the States (constants in config/StateMachineConfig.h)
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"

namespace StateCommon
{
    using namespace Constants::StateMachine::Common;
    using namespace Constants::StateMachine::Elevator;

    static constexpr int kQtrPosMax = (QTR::N - 1) * 1000; // 6000 (indice 6 * 1000)

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
        // Rear has fewer sensors: rescale to the front 0..kQtrPosMax range.
        const int   rearPosMax      = (qtrRear.size() - 1) * 1000;
        const float mirroredRearPos = (float)(rearPosMax - qtrRear.getPosition()) * kQtrPosMax / rearPosMax;
        const float error = kCornerSetpoint - mirroredRearPos;
        return constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }
}
