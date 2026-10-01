#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BeansState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx, bool& transitionToBeansGoBack, bool& transitionToPoolsGoBack, bool& transitionToStop) {
        transitionToBeansGoBack = false;
        transitionToPoolsGoBack = false;
        transitionToStop = false;

        static constexpr uint32_t kLostLineTimeoutMs = 1200;

        if (vision.hasCriticalError()) {
            vision.stop();
            transitionToStop = true;
            return;
        }

        switch (action_stage) {
            case 0: {
                if (cornerRIGHTDetected) {
                    odomMove_.stop();
                    action_start_time = now;
                    action_stage = 1;
                    return;
                }

                if (!onLine) {
                    if (action_start_time == 0)
                        action_start_time = now;

                    odomMove_.backward(58.0f);

                    if ((now - action_start_time) >= kLostLineTimeoutMs) {
                        vision.stop();
                        transitionToPoolsGoBack = true;
                    }
                    return;
                }

                action_start_time = 0;

                const int error = 2200 - qtrFront.getPosition();
                const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
                odomMove_.setTranslation(+50.0f, corr);
                break;
            }

            case 1: {
                odomMove_.stop();
                if ((now - action_start_time) >= 1000) {
                    action_start_time = 0;
                    transitionToBeansGoBack = true;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};