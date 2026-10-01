/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.3- Start State 
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class LookForLineBackwardsState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected, bool& transitionToBenefitsStartCorner) {
        transitionToBenefitsStartCorner = false;

        static constexpr float kVelocity = 0.48f;
        static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

        switch (action_stage) {
            // ── Stage 0: Buscar línea trasera completa ──────────────────────
            case 0: {
                if (backDetected) {
                    transitionToBenefitsStartCorner = true;
                    return;
                }

                if (backLeftDetected && !backRightDetected) {
                    action_stage = 1;
                    action_start_time = now;
                    return;
                }

                if (backRightDetected && !backLeftDetected) {
                    action_stage = 2;
                    action_start_time = now;
                    return;
                }

                odomMove_.backward(kBaseSpeed);
                break;
            }

            // ── Stage 1: Girar derecha por 500 ms ──────────────────────────
            case 1: {
                if (backDetected) {
                    transitionToBenefitsStartCorner = true;
                    return;
                }

                LARC.right(kVelocity);

                if ((now - action_start_time) >= 500) {
                    action_stage = 0;
                }
                break;
            }

            // ── Stage 2: Girar izquierda por 500 ms ───────────────────────
            case 2: {
                if (backDetected) {
                    transitionToBenefitsStartCorner = true;
                    return;
                }

                LARC.left(kVelocity);

                if ((now - action_start_time) >= 500) {
                    action_stage = 0;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};