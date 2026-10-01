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

            case 1: {
                LARC.setTranslation(vx, -0.48f);

                if (cornerRIGHTDetected) {
                    action_stage = 2;
                }
                break;
            }

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
