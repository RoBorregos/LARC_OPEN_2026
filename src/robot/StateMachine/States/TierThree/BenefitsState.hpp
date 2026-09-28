/*
*@author:  Ximena Patricia García Magdaleno
* BenefitsState.hpp
* State Machine Tier Three.3- Benefits State
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BenefitsState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        lastRequestMs = 0;

        vision.setBenefitsOnRequest(true);
        vision.closeBenefit();
        vision.resetGuards();
        vision.startBenefits();
    }

    void update(uint32_t now, bool cornerRIGHTDetected, float vx, bool onLine, bool& transitionToStop) {
        transitionToStop = false;

        // Re-request until the Orin confirms it is actually in BENEFITS. One
        // dropped byte would otherwise leave us driving the corner with the
        // rear camera still idle, and nothing would ever say so.
        if (!vision.inBenefitsPhase() && (now - lastRequestMs) >= kRequestRetryMs) {
            lastRequestMs = now;
            vision.resetGuards();
            vision.startBenefits();
        }

        if (!vision.inBenefitsPhase() || !vision.isBenefitsRunning()) {
            LARC.brake();
            return;
        }

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
                    LARC.brake();
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            // Stop for 1000 ms, then open only with a centred detection.
            case 2: {
                LARC.brake();

                if ((now - action_start_time) >= 1000) {
                    if (vision.inBenefitsPhase() && vision.benefitSeen()) {
                        vision.openBenefit();
                        action_start_time = now;
                        action_stage = 3;
                    }
                }
                break;
            }
            case 3:
                LARC.brake();
                if ((now - action_start_time) >= Constants::ServoConfig::kBenefitOpenMs)
                    transitionToStop = true;
                break;
        }
    }

private:
    static constexpr uint32_t kRequestRetryMs = 1000;

    uint8_t  action_stage = 0;
    uint32_t action_start_time = 0;
    uint32_t lastRequestMs = 0;
};
