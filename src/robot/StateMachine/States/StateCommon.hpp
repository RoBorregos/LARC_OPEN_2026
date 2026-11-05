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

    // QTR -> vx correction; the states apply -correction. Used for qtrFront and qtrRear
    // (rear is mirrored, so the same sign works). Call every loop; it only recomputes every kFrontLinePeriodMs.
    class LineCorrection
    {
    public:
        LineCorrection(QTR& qtr, float setpoint, bool teleplot = false)
            : qtr_(qtr), setpoint_(setpoint), teleplot_(teleplot) {}

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

            const int pos = qtr_.getPosition();
            float errMm = 0.0f;

            if (onLine)
            {
                lost_      = false;
                errMm      = (setpoint_ - pos) / 1000.0f * kQtrPitchMm;
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

            if (teleplot_)
            {
                plot(F("sp"), now, setpoint_, 0);
                plot(F("pos"), now, pos, 0);
                plot(F("errMm"), now, errMm, 2);
                plot(F("u"), now, pd_.command(), 4);
                plot(F("out"), now, pd_.output(), 4);
                plot(F("onLine"), now, onLine ? 1.0f : 0.0f, 0);

                const uint16_t* norm = qtr_.getNorm();
                for (uint8_t i = 0; i < qtr_.size(); i++)
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

        // No correction: slews the output to 0 and clears the PD memory, so update()
        // resumes without a derivative kick.
        float pause(uint32_t now)
        {
            if (started_ && (now - lastTickMs_) < kFrontLinePeriodMs)
                return pd_.output();

            const bool  gap = !started_ || (now - lastTickMs_) > kMaxGapMs;
            const float dt  = (gap ? kFrontLinePeriodMs : (now - lastTickMs_)) / 1000.0f;
            started_    = true;
            lastTickMs_ = now;
            lost_       = false;
            pd_.hold(0.0f, dt);
            return pd_.output();
        }

    private:
        static constexpr uint32_t kMaxGapMs = 5 * kFrontLinePeriodMs;
        static constexpr LinePD::Gains kGains = {
            kFrontLineKp, kFrontLineKi, kFrontLineKd, kFrontLineDTauS, kCornerCorrMax, kFrontLineSlewPerS};

        QTR&     qtr_;
        float    setpoint_;
        bool     teleplot_;
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

    inline bool rearOnLine() { return qtrRear.onLine(kRearOnLineThreshold); }
}
