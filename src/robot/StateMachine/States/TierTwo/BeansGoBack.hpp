#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BeansGoBackState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool frontLeftDetected, bool onLine, float vx, bool& transitionToBeans) {
        transitionToBeans = false;

        switch (action_stage) {
            case 0: {
                vision.stop();
                vision.clearErrors();
                elevator.ElevatorPosition(0);
                odomMove_.stop();
                action_start_time = now;
                action_stage = 1;
                return;
            }

            case 1: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();
                action_start_time = now;
                action_stage = 2;
                return;
            }

            case 2: {
                odomMove_.stop();
            }

            case 3: {
                elevator.ElevatorPosition(0);

                if (frontLeftDetected) {
                    odomMove_.stop();
                    action_start_time = now;
                    action_stage = 4;
                    return;
                }

                if (!onLine) {
                    odomMove_.backward(55.0f);
                } else {
                    const int error = 2900 - qtrFront.getPosition();
                    const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
                    odomMove_.setTranslation(-50.0f, corr);
                }
                return;
            }

            case 4: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();

                if ((now - action_start_time) >= 8000) {
                    vision.startBeans();
                    transitionToBeans = true;
                }
                return;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};