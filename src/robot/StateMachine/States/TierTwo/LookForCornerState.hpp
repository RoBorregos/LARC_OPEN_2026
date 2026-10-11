/*
*@author:  Ximena Patricia García Magdaleno
* LookForCornerState.hpp
* State Machine Tier Two.1- Look For Corner State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class LookForCornerState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        beansRequestMs = 0;
        cornerCorrFiltered = 0.0f;
    }

    void update(uint32_t now, bool cornerLEFTDetected, bool onLine,
                bool& transitionToBeans, bool& transitionToStop) {
        using namespace StateCommon;
        transitionToBeans = false;
        transitionToStop = false;

        static constexpr uint32_t kBeansRetryMs = 1000; // re-send START_BEANS while waiting
        static constexpr uint32_t kSoftStartMs  = 500;

        switch (action_stage) {
            // ── Stage 0: Buscar esquina LEFT con corrección de línea ────────────
            case 0: {
                if (cornerLEFTDetected) {
                    LARC.stop();
                    vision.startBeans();
                    action_stage = 1;
                    action_start_time = now;
                    return;
                }

                const float corrTarget = frontCornerCorrTarget(onLine);
                cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

                LARC.setTranslation(-cornerCorrFiltered, kVelocity);
                break;
            }

            // ── Stage 1: Stop hasta que la Orin reporte BEANS corriendo ───────
            // A startup fault ends the run instead of alternating STOP and
            // START_BEANS forever.
            case 1: {
                LARC.stop();

                if (vision.hasError()) {
                    Serial.println(F("[SM] BEANS vision failed; stopping run"));
                    vision.resetGuards();
                    vision.stop();
                    transitionToStop = true;
                    return;
                }

                if (vision.inBeansPhase() && vision.isBeansRunning()) {
                    action_stage = 2;
                    action_start_time = now;
                } else if (beansRequestMs == 0 || (now - beansRequestMs) >= kBeansRetryMs) {
                    beansRequestMs = now;
                    vision.resetGuards();
                    vision.startBeans();
                    Serial.println(F("[SM] waiting for fresh BEANS vision"));
                }
                break;
            }

            // ── Stage 2: Stop por kSoftStartMs antes de transicionar ────────
            case 2: {
                LARC.stop();

                if ((now - action_start_time) >= kSoftStartMs) {
                    transitionToBeans = true;
                }
                break;
            }
        }
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;
    uint32_t beansRequestMs = 0;
    float cornerCorrFiltered = 0.0f;
};
