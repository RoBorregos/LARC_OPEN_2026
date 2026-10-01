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
        cornerCorrFiltered = 0.0f;
    }

    void update(uint32_t now, bool cornerLEFTDetected, bool onLine, bool& transitionToBeans) {
        using namespace StateCommon;
        transitionToBeans = false;

        static constexpr uint32_t kCornerStopMs = 8200; //Para que vision empiece
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

            // ── Stage 1: Stop por kCornerStopMs (esperando a vision) ──────────
            case 1: {
                LARC.stop();

                if ((now - action_start_time) >= kCornerStopMs) {
                    action_stage = 2;
                    action_start_time = now;
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
    float cornerCorrFiltered = 0.0f;
};
