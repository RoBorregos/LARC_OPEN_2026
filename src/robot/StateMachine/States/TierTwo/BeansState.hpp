/*
*@author:  Ximena Patricia García Magdaleno
* BeansState.hpp
* State Machine Tier Two.2- Beans State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class BeansState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        cornerCorrFiltered = 0.0f;
    }

    void update(uint32_t now, bool cornerRIGHTDetected, bool onLine,
                bool& transitionToPoolsGoBack, bool& transitionToStop) {
        using namespace StateCommon;
        transitionToPoolsGoBack = false;
        transitionToStop = false;

        static constexpr uint32_t kLostLineTimeoutMs = 1200;

        // Check critical error from vision
        if (vision.hasCriticalError()) {
            vision.stop();
            transitionToStop = true;
            return;
        }

        switch (action_stage) {
            // ── Stage 0: Búsqueda y recolección de beans ────────────────────
            case 0: {
                if (cornerRIGHTDetected) {
                    LARC.stop();
                    action_start_time = now;
                    action_stage = 1;
                    return;
                }

                /*
                if (!onLine)
                {
                    if (action_start_time == 0)
                        action_start_time = now;

                    LARC.backward(0.30f);

                    if ((now - action_start_time) >= kLostLineTimeoutMs)
                    {
                        vision.stop();
                        transitionToPoolsGoBack = true;
                    }

                    return;
                }*/

                action_start_time = 0;

                const float corrTarget = frontCornerCorrTarget(onLine);
                cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

                LARC.setTranslation(-cornerCorrFiltered, -kVelocity);
                break;
            }

            // ── Stage 1: Stop por 1000 ms después de detectar RIGHT ────────
            case 1: {
                LARC.stop();
                if ((now - action_start_time) >= 1000) {
                    action_start_time = 0;
                    transitionToPoolsGoBack = true;
                }
                break;
            }
        }
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;
    float cornerCorrFiltered = 0.0f;
};
