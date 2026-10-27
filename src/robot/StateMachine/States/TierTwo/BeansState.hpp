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
        lineCorr_.reset();
    }

    void update(uint32_t now, bool cornerRIGHTDetected, bool onLine,
                bool& transitionToPoolsGoBack, bool& transitionToStop) {
        using namespace StateCommon;
        transitionToPoolsGoBack = false;
        namespace cfg = Constants::StateMachine::Beans;
        transitionToStop = false;

        // Check critical error from vision
        if (vision.hasCriticalError()) {
            vision.stop();
            transitionToStop = true;
            return;
        }

        // No fresh BEANS results right now (startup or a stall): wait in place
        // rather than drive past balls nobody is sorting.
        if (!vision.inBeansPhase() || !vision.isBeansRunning()) {
            LARC.stop();
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

                    LARC.backward(kManeuverSpeed);

                    if ((now - action_start_time) >= cfg::kLostLineTimeoutMs)
                    {
                        vision.stop();
                        transitionToPoolsGoBack = true;
                    }

                    return;
                }*/

                action_start_time = 0;

                const float corrMax = cfg::kCorrMaxRatio * cfg::kVelocity;
                const float corr = constrain(lineCorr_.update(now, onLine), -corrMax, corrMax);
                LARC.setTranslation(-corr, -cfg::kVelocity);
                break;
            }

            // ── Stage 1: Stop por 1000 ms después de detectar RIGHT ────────
            case 1: {
                LARC.stop();
                if ((now - action_start_time) >= cfg::kCornerStopMs) {
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
    StateCommon::FrontLineCorrection lineCorr_;
};
