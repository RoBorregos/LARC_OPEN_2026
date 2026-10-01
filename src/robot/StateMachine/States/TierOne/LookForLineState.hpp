/*
*@author:  Ximena Patricia García Magdaleno
* LookForLineState.hpp
* State Machine Tier One.3- Look For Line State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class LookForLineState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        lfCorrecting = false;
        lfCorrectionDir = 0;
        lfCorrectionStartMs = 0;
    }

    void update(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected,
                bool onLine, bool& transitionToCorner) {
        transitionToCorner = false;

        // ── Stage 0: retroceder 200 ms ────────────────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(0.30f);

            if ((now - action_start_time) >= 200) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 1: retroceder 200 ms más ────────────────────────────────────
        if (action_stage == 1) {
            LARC.backward(0.30f);

            if ((now - action_start_time) >= 200) {
                action_stage = 2;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 2: avanzar 300 ms ───────────────────────────────────────────
        if (action_stage == 2) {
            LARC.forward(0.30f);

            if ((now - action_start_time) >= 300) {
                action_stage = 3;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 3: búsqueda normal ───────────────────────────────────────────
        static constexpr uint32_t kBorderCorrectMs = 150;
        static constexpr uint16_t kTofBorderMm     = 150;

        const bool realBorderLeft  = leftDetected  && tofLeft.isValid()  && tofLeft.getDistanceMm()  > kTofBorderMm;
        const bool realBorderRight = rightDetected && tofRight.isValid() && tofRight.getDistanceMm() > kTofBorderMm;

        if (frontDetected && onLine) {
            lfCorrecting        = false;
            lfCorrectionDir     = 0;
            lfCorrectionStartMs = 0;
            Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
            LARC.stop();
            transitionToCorner = true;
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

        if (realBorderLeft && !rightDetected) {
            lfCorrecting        = true;
            lfCorrectionDir     = +1;
            lfCorrectionStartMs = now;
            LARC.right(0.30f);
            return;
        }

        if (realBorderRight && !leftDetected) {
            lfCorrecting        = true;
            lfCorrectionDir     = -1;
            lfCorrectionStartMs = now;
            LARC.left(0.30f);
            return;
        }

        LARC.forward(0.30f);
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;

    bool     lfCorrecting        = false;
    int8_t   lfCorrectionDir     = 0;
    uint32_t lfCorrectionStartMs = 0;
};
