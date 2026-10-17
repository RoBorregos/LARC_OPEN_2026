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
        const float error = kQtrFrontSetpoint - qtrFront.getPosition();
        // Shrinking the error by the deadband (instead of zeroing it) keeps the correction continuous.
        const float shrunk = (error > 0.0f) ? max(error - kFrontCornerDeadband, 0.0f)
                                            : min(error + kFrontCornerDeadband, 0.0f);
        return constrain(shrunk * kFrontCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }

    // qtrRear is wired mirrored to qtrFront
    inline float rearCornerCorrTarget()
    {
        if (!qtrRear.onLine())
            return 0.0f;
        // Mirrored sign; rear has fewer sensors, so the error is rescaled to the
        // front 0..kQtrPosMax range to share kCornerKp.
        const int   rearPosMax = (qtrRear.size() - 1) * 1000;
        const float error = ((float)qtrRear.getPosition() - kQtrRearSetpoint) * kQtrPosMax / rearPosMax;
        return constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }
}
