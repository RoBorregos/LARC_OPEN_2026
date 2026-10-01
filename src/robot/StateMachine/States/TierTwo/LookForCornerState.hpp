#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class LookForCornerState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
    }

    void update(uint32_t now, bool cornerLEFTDetected, float vx, bool& transitionToBeans) {
        transitionToBeans = false;

        static constexpr uint32_t kCornerStopMs = 8200;  // Para que vision empiece
        static constexpr uint32_t kSoftStartMs = 500;

        switch (action_stage) {
            // ── Stage 0: Buscar esquina LEFT y esperar ─────────────────────────
            case 0: {
                if (cornerLEFTDetected) {
                    odomMove_.stop();
                    vision.startBeans();
                    action_stage = 1;
                    action_start_time = now;
                    return;
                }
                // Avanzar hacia atrás con corrección de línea
                const int error = 2900 - qtrFront.getPosition();
                const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
                odomMove_.setTranslation(-50.0f, corr);
                break;
            }

            // ── Stage 1: Stop por kCornerStopMs (esperando a vision) ──────────
            case 1: {
                odomMove_.stop();

                if ((now - action_start_time) >= kCornerStopMs) {
                    action_stage = 2;
                    action_start_time = now;
                }
                break;
            }

            // ── Stage 2: Stop por kSoftStartMs antes de transicionar ────────
            case 2: {
                odomMove_.stop();

                if ((now - action_start_time) >= kSoftStartMs) {
                    transitionToBeans = true;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
};