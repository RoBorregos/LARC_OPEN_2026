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

        benefitRequestMs = 0;
        benefitCentredMs = 0;
        benefitColour = VisionLink::kNoBenefit;
        benefitArmed = true;
        benefitOpened = false;
        benefitOpenedMs = 0;
        benefitLastSeenMs = 0;
        benefitApproachMs = 0;
    }

    void update(uint32_t now, bool cornerRIGHTDetected, bool& transitionToStop) {
        using namespace StateCommon;
        using namespace Constants::ServoConfig;
        namespace cfg = Constants::StateMachine::Benefits;
        transitionToStop = false;

        // Doors open only through openBenefit(); the stream just reports the box.
        vision.setBenefitsOnRequest(true);

        if (vision.hasError()) {
            LARC.stop();
            vision.closeBenefit();
            // Serial.println(F("[SM] BENEFITS vision failed; stopping run"));
            vision.resetGuards();
            vision.stop();
            transitionToStop = true;
            return;
        }

        // Wait in place until the Orin reports fresh BENEFITS results,
        // re-sending START_BENEFITS every second.
        if (!vision.inBenefitsPhase() || !vision.isBenefitsRunning()) {
            LARC.stop();
            benefitCentredMs = now;
            if (benefitRequestMs == 0 || (now - benefitRequestMs) >= cfg::kVisionRetryMs) {
                vision.resetGuards();
                vision.startBenefits();
                benefitRequestMs = now;
                // Serial.println(F("[SM] waiting for fresh BENEFITS vision"));
            }
            return;
        }

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

            // ── Stage 1: Avanzar hasta la esquina RIGHT, parando en cada caja ─
            case 1: {
                const float corrTarget = rearCornerCorrTarget();
                rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

                LARC.setTranslation(rearCorrFiltered, -kVelocity);

                // Re-arm only once the box has FULLY left the camera (no ROI sees
                // it) for kBenefitRearmMs, so a box sliding out slowly, or
                // flickering between CENTER and RIGHT, cannot cause a second stop.
                if (vision.benefitVisible())
                    benefitLastSeenMs = now;
                else if (now - benefitLastSeenMs >= kBenefitRearmMs)
                    benefitArmed = true;

                // The box must stay centred for kBenefitApproachMs before we stop,
                // so a one-frame CENTER while it is still entering is ignored.
                if (!vision.benefitSeen())
                    benefitApproachMs = 0;
                else if (benefitApproachMs == 0)
                    benefitApproachMs = now;

                if (cornerRIGHTDetected) {
                    LARC.stop();
                    action_start_time = now;
                    action_stage = 2;
                }
                // The 3 benefits ROIs see the same colour: stop, then open
                else if (benefitArmed && vision.benefitSeen() && benefitApproachMs != 0 &&
                         (now - benefitApproachMs) >= kBenefitApproachMs) {
                    LARC.stop();
                    benefitArmed = false;
                    benefitApproachMs = 0;
                    benefitOpened = false;
                    benefitCentredMs = now;
                    benefitColour = vision.seenBenefit();
                    action_start_time = now;
                    action_stage = 3;
                }
                break;
            }

            // ── Stage 2: Stop por 1000 ms antes de transicionar a STOP ────
            case 2: {
                LARC.stop();

                if ((now - action_start_time) >= cfg::kCornerStopMs) {
                    transitionToStop = true;
                }
                break;
            }

            // ── Stage 3: Parado en una caja: confirmar color, abrir, cerrar ──
            case 3: {
                LARC.stop();

                // Require the same centred colour for kBenefitConfirmMs while stopped.
                if (!benefitOpened && (!vision.benefitSeen() || vision.seenBenefit() != benefitColour)) {
                    benefitCentredMs = now;
                    benefitColour = vision.seenBenefit();
                }
                if (!benefitOpened && vision.benefitSeen() && (now - benefitCentredMs) >= kBenefitConfirmMs) {
                    vision.openBenefit();
                    benefitOpened = vision.benefitHeld();
                    benefitOpenedMs = now;
                }

                if (benefitOpened && vision.benefitHeld() && (now - benefitOpenedMs) >= kBenefitOpenMs)
                    vision.closeBenefit();

                if ((benefitOpened && (now - benefitOpenedMs) >= kBenefitStopMs) ||
                    (!benefitOpened && (now - action_start_time) >= kBenefitStopMs)) {
                    vision.closeBenefit();
                    benefitLastSeenMs = now; // the box must vanish for kBenefitRearmMs from here
                    // No door opened (colour never held): stay armed, so the robot
                    // stops again once this box is properly centred.
                    if (!benefitOpened)
                        benefitArmed = true;
                    action_stage = 1;
                }
                break;
            }
        }
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;
    float rearCorrFiltered = 0.0f;

    uint32_t benefitRequestMs = 0;
    uint32_t benefitCentredMs = 0;
    uint8_t  benefitColour = VisionLink::kNoBenefit;
    bool     benefitArmed = true;
    bool     benefitOpened = false;
    uint32_t benefitOpenedMs = 0;
    uint32_t benefitLastSeenMs = 0; // last time any part of a box was in view while driving
    uint32_t benefitApproachMs = 0; // when the box became centred while driving, 0 = not centred
};
