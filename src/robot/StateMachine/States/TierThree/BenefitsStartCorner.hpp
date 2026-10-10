/*
*@author:  Ximena Patricia García Magdaleno
* BenefitsStartCorner.hpp
* State Machine Tier Three.2- Benefits Start Corner State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class BenefitsStartCornerState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        rearCorrFiltered = 0.0f;
    }

    void update(uint32_t now, bool cornerLeftDetected, bool& transitionToBenefits) {
        using namespace StateCommon;
        transitionToBenefits = false;

        switch (action_stage) {
            // ── Stage 0: Buscar esquina LEFT con corrección de qtrRear ──────
            case 0: {
                if (cornerLeftDetected) {
                    LARC.brake();
                    action_start_time = now;
                    action_stage = 1;
                    return;
                }

                const float corrTarget = rearCornerCorrTarget();
                rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

                LARC.setTranslation(rearCorrFiltered, kVelocity);
                break;
            }

            // ── Stage 1: Stop por 1000 ms antes de transicionar ──────────
            case 1: {
                LARC.stop();
                if ((now - action_start_time) >= 1000) {
                    transitionToBenefits = true;
                }
                break;
            }
        }
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;
    float rearCorrFiltered = 0.0f;
};
