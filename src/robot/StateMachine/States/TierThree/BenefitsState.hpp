/*
*@author:  Ximena Patricia García Magdaleno
* BenefitsState.hpp
* State Machine Tier Three.2- Benefits State
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BenefitsState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool cornerRIGHTDetected, float vx, bool onLine, bool& transitionToStop) {
        transitionToStop = false;

        switch (action_stage) {
            // ── Stage 0: Decidir si ya estamos en la esquina RIGHT ──────────
            case 0: {
                if (!cornerRIGHTDetected) {
                    action_stage = 1;
                } else {
                    LARC.brake();
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 1: Avanzar hasta detectar la esquina RIGHT ────────────
            case 1: {
                LARC.setTranslation(vx, -0.48f);

                if (cornerRIGHTDetected) {
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 2: Brake por 1000 ms antes de transicionar a STOP ────
            case 2: {
                LARC.brake();

                if ((now - action_start_time) >= 1000) {
                    transitionToStop = true;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};
