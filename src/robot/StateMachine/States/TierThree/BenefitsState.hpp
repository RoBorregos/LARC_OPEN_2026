/*
*@author:  Ximena Patricia García Magdaleno
* BenefitsState.hpp
* State Machine Tier Three.3- Benefits State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class BenefitsState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        rearCorrFiltered = 0.0f;
    }

    void update(uint32_t now, bool cornerRIGHTDetected, bool& transitionToStop) {
        using namespace StateCommon;
        transitionToStop = false;

        switch (action_stage) {
            // ── Stage 0: Decidir si ya estamos en la esquina RIGHT ──────────
            case 0: {
                if (!cornerRIGHTDetected) {
                    action_stage = 1;
                } else {
                    LARC.stop();
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 1: Avanzar hasta detectar la esquina RIGHT ────────────
            case 1: {
                const float corrTarget = rearCornerCorrTarget();
                rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

                LARC.setTranslation(rearCorrFiltered, -kVelocity);

                // Here goes the rutine
                if (cornerRIGHTDetected) {
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 2: Stop por 1000 ms antes de transicionar a STOP ────
            case 2: {
                LARC.stop();

                if ((now - action_start_time) >= 1000) {
                    transitionToStop = true;
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
