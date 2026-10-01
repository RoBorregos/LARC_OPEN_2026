/*
*@author:  Ximena Patricia García Magdaleno
* LookForLineBackwards.hpp
* State Machine Tier Three.1- Look For Line Backwards State
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"

class LookForLineBackwardsState {
public:
    void begin() {
        lfCorrecting = false;
        lfCorrectionDir = 0;
        lfCorrectionStartMs = 0;
        backLineArmed = false;
        backLineArmedMs = 0;
    }

    void update(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected,
                bool& transitionToBenefitsStartCorner) {
        transitionToBenefitsStartCorner = false;

        static constexpr uint32_t kBorderCorrectMs = 150;
        static constexpr uint16_t kTofBorderMm     = 150;
        static constexpr uint32_t kBackLineArmDelayMs = 700;

        const bool realBorderLeft  = backLeftDetected  && tofLeft.isValid()  && tofLeft.getDistanceMm()  > kTofBorderMm;
        const bool realBorderRight = backRightDetected && tofRight.isValid() && tofRight.getDistanceMm() > kTofBorderMm;

        if (backDetected && !backLineArmed) {
            backLineArmed   = true;
            backLineArmedMs = now;
        }

        const bool armDelayElapsed = backLineArmed && (now - backLineArmedMs) >= kBackLineArmDelayMs;

        // Off-line floor already reads norm ~470 on qtrRear, so the default 200 is useless here.
        if (armDelayElapsed && qtrRear.onLine(Constants::QTRCalibration::kBinaryThreshold)) {
            lfCorrecting        = false;
            lfCorrectionDir     = 0;
            lfCorrectionStartMs = 0;
            Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
            LARC.stop();
            transitionToBenefitsStartCorner = true;
            return;
        }

        if (lfCorrecting) {
            if ((now - lfCorrectionStartMs) < kBorderCorrectMs) {
                if (lfCorrectionDir < 0)
                    LARC.left(0.30f);
                else
                    LARC.right(0.30f);
                return;
            }
            lfCorrecting = false;
            LARC.forward(0.30f);
            return;
        }

        if (realBorderLeft && !backRightDetected) {
            lfCorrecting        = true;
            lfCorrectionDir     = +1;
            lfCorrectionStartMs = now;
            LARC.right(0.30f);
            return;
        }

        if (realBorderRight && !backLeftDetected) {
            lfCorrecting        = true;
            lfCorrectionDir     = -1;
            lfCorrectionStartMs = now;
            LARC.left(0.30f);
            return;
        }

        LARC.backward(0.30f);
    }

private:
    bool     lfCorrecting        = false;
    int8_t   lfCorrectionDir     = 0;
    uint32_t lfCorrectionStartMs = 0;

    bool     backLineArmed   = false;
    uint32_t backLineArmedMs = 0;
};
