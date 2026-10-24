/*
*@author:  Ximena Patricia García Magdaleno
* StateCommon.hpp
* Shared QTR corrections for the States (constants in config/StateMachineConfig.h)
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "LinePD.hpp"
#include "robot/instances/instances.hpp"

namespace StateCommon
{
    using namespace Constants::StateMachine::Common;
    using namespace Constants::StateMachine::Elevator;

    static constexpr int kQtrPosMax = (QTR::N - 1) * 1000; // 6000 (indice 6 * 1000)

    // qtrFront -> vx correction (same sign as the old P: the states apply -correction).
    // Call every loop; it only recomputes every kFrontLinePeriodMs.
    class FrontLineCorrection
    {
    public:
        void reset()
        {
            pd_.reset();
            started_   = false;
            lost_      = false;
            lastErrMm_ = 0.0f;
        }

        float update(uint32_t now, bool onLine)
        {
            if (started_ && (now - lastTickMs_) < kFrontLinePeriodMs)
                return pd_.output();

            // Long gap (e.g. BEANS waiting for vision with the robot stopped): restart from rest.
            if (started_ && (now - lastTickMs_) > kMaxGapMs)
            {
                pd_.reset();
                started_ = false;
            }

            const float dt = (started_ ? (now - lastTickMs_) : kFrontLinePeriodMs) / 1000.0f;
            started_    = true;
            lastTickMs_ = now;

            const int pos = qtrFront.getPosition();
            float errMm = 0.0f;

            if (onLine)
            {
                lost_      = false;
                errMm      = (kQtrFrontSetpoint - pos) / 1000.0f * kQtrPitchMm;
                lastErrMm_ = errMm;
                // Shrinking the error by the deadband (instead of zeroing it) keeps the correction continuous.
                const float shrunk = (errMm > 0.0f) ? max(errMm - kFrontLineDeadbandMm, 0.0f)
                                                    : min(errMm + kFrontLineDeadbandMm, 0.0f);
                pd_.update(shrunk, dt);
            }
            else
            {
                if (!lost_)
                {
                    lost_        = true;
                    lostSinceMs_ = now;
                }
                const bool recovering = fabsf(lastErrMm_) >= kFrontLostEdgeMm &&
                                        (now - lostSinceMs_) < kFrontLostRecoverMs;
                const float push = (lastErrMm_ > 0.0f) ? kCornerCorrMax : -kCornerCorrMax;
                pd_.hold(recovering ? push : 0.0f, dt);
            }

            if (Constants::StateMachine::Debug::kFrontLineTeleplot)
            {
                plot(F("sp"), now, kQtrFrontSetpoint, 0);
                plot(F("pos"), now, pos, 0);
                plot(F("errMm"), now, errMm, 2);
                plot(F("u"), now, pd_.command(), 4);
                plot(F("out"), now, pd_.output(), 4);
                plot(F("onLine"), now, onLine ? 1.0f : 0.0f, 0);

                const uint16_t* norm = qtrFront.getNorm();
                for (uint8_t i = 0; i < qtrFront.size(); i++)
                {
                    Serial.print(F(">n"));
                    Serial.print(i);
                    Serial.print(':');
                    Serial.print(now);
                    Serial.print(':');
                    Serial.println(norm[i]);
                }
            }
            return pd_.output();
        }

    private:
        static constexpr uint32_t kMaxGapMs = 5 * kFrontLinePeriodMs;
        static constexpr LinePD::Gains kGains = {
            kFrontLineKp, kFrontLineKi, kFrontLineKd, kFrontLineDTauS, kCornerCorrMax, kFrontLineSlewPerS};

        LinePD   pd_{kGains};
        bool     started_     = false;
        bool     lost_        = false;
        uint32_t lastTickMs_  = 0;
        uint32_t lostSinceMs_ = 0;
        float    lastErrMm_   = 0.0f;

        // Teleplot line: ">name:timestampMs:value"
        static void plot(const __FlashStringHelper* name, uint32_t now, float value, uint8_t decimals)
        {
            Serial.print('>');
            Serial.print(name);
            Serial.print(':');
            Serial.print(now);
            Serial.print(':');
            Serial.println(value, decimals);
        }
    };

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
