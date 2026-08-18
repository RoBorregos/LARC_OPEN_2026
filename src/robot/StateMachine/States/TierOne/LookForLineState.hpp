/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.2- Start State 
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
 
        static constexpr uint32_t kBorderCorrectMs = 150;
        static constexpr float kTofBorderCm = 15.0f;
 
        // ── Stage 0: retroceder 200 ms ────────────────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;
 
            odomMove_.backward(180.0f);
 
            if ((now - action_start_time) >= 200) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }
 
        // ── Stage 1: retroceder 200 ms más ────────────────────────────────────
        if (action_stage == 1) {
            odomMove_.backward(200.0f);
 
            if ((now - action_start_time) >= 200) {
                action_stage = 2;
                action_start_time = now;
            }
            return;
        }
 
        // ── Stage 2: avanzar 300 ms ───────────────────────────────────────────
        if (action_stage == 2) {
            odomMove_.forward(120.0f);
 
            if ((now - action_start_time) >= 300) {
                action_stage = 3;
                action_start_time = now;
            }
            return;
        }
 
        // ── Stage 3: búsqueda normal ───────────────────────────────────────────
        const bool realBorderLeft = leftDetected && tofLeft.isValid() &&
                                    tofLeft.getDistanceCm() > kTofBorderCm;
        const bool realBorderRight = rightDetected && tofRight.isValid() &&
                                     tofRight.getDistanceCm() > kTofBorderCm;
 
        if (frontDetected && onLine) {
            lfCorrecting = false;
            lfCorrectionDir = 0;
            lfCorrectionStartMs = 0;
            Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
            odomMove_.stop();
            transitionToCorner = true;
            return;
        }
 
        if (lfCorrecting) {
            if ((now - lfCorrectionStartMs) < kBorderCorrectMs) {
                if (lfCorrectionDir < 0)
                    odomMove_.left(55.0f);
                else
                    odomMove_.right(55.0f);
                return;
            }
            lfCorrecting = false;
            odomMove_.forward(50.0f);
            return;
        }
 
        if (realBorderLeft && !rightDetected) {
            lfCorrecting = true;
            lfCorrectionDir = +1;
            lfCorrectionStartMs = now;
            odomMove_.right(55.0f);
            return;
        }
 
        if (realBorderRight && !leftDetected) {
            lfCorrecting = true;
            lfCorrectionDir = -1;
            lfCorrectionStartMs = now;
            odomMove_.left(55.0f);
            return;
        }
 
        odomMove_.forward(50.0f);
    }
 
    void reset() {
        lfCorrecting = false;
        lfCorrectionDir = 0;
        lfCorrectionStartMs = 0;
    }
 
private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
 
    bool lfCorrecting = false;
    int8_t lfCorrectionDir = 0;
    uint32_t lfCorrectionStartMs = 0;
};
 
