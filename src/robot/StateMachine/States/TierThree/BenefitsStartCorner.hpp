/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.3- Start State 
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BenefitsStartCornerState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool cornerLeftDetected, float vx, bool onLine, bool& transitionToBenefits) {
        transitionToBenefits = false;

        static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

        switch (action_stage) {
            // ── Stage 0: Buscar esquina LEFT ────────────────────────────────
            case 0: {
                if (cornerLeftDetected) {
                    LARC.brake();
                    action_start_time = now;
                    action_stage = 1;
                    return;
                }

                if (!onLine) {
                    LARC.left(kBaseSpeed);
                    return;
                }

                LARC.setTranslation(vx, 0.48f);
                break;
            }

            // ── Stage 1: Brake por 1000 ms antes de transicionar ──────────
            case 1: {
                LARC.brake();
                if ((now - action_start_time) >= 1000) {
                    transitionToBenefits = true;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};