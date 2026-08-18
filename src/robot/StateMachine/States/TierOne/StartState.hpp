/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.1- Start State 
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class StartState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        lastLimitPressed = false;
    }

    void update(uint32_t now, bool& transitionToPool) {
        transitionToPool = false;

        vision.stop();

        const bool limitPressed = (digitalRead(Pins::kLimitSwitch) == HIGH);

        if (limitPressed != lastLimitPressed) {
            if (limitPressed)
                Serial.println("LIMIT SWITCH PRESIONADO");
            else
                Serial.println("LIMIT SWITCH LIBERADO");
            lastLimitPressed = limitPressed;
        }

        switch (action_stage) {
            // ── Stage 0: Subir por 12000 ms ────────────────────────────────────
            case 0: {
                if (limitPressed) {
                    // Limit presionado durante la subida → interrumpir y bajar
                    elevator.ElevatorPosition(0);
                    odomMove_.stop();
                    action_start_time = now;
                    action_stage = 1;
                } else {
                    elevator.ElevatorPosition(2);
                    odomMove_.stop();

                    if ((now - action_start_time) >= 12000) {
                        // Subida completa → pasar al elevador stop
                        action_start_time = now;
                        action_stage = 4;
                    }
                }
                break;
            }

            // ── Stage 1: Bajar mientras limit esté presionado ───────────────────
            case 1: {
                elevator.ElevatorPosition(1);
                odomMove_.stop();

                if (!limitPressed) {
                    // Limit suelto → esperar 2000 ms antes de reintentar subida
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 2: Esperar 2000 ms con elevador parado ──────────────────
            case 2: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();

                if ((now - action_start_time) >= 2000) {
                    // Reintentar subida desde cero
                    action_start_time = now;
                    action_stage = 0;
                }
                break;
            }

            // ── Stage 4: Elevador stop 1500 ms ───────────────────────────────
            case 4: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();
                if ((now - action_start_time) >= 1500) {
                    action_start_time = now;
                    action_stage = 5;
                }
                break;
            }

            // ── Stage 5: Avanzar y transicionar a POOL ───────────────────────
            case 5: {
                elevator.ElevatorPosition(0);
                odomMove_.forward(50.0f);

                static constexpr uint32_t kStartIgnoreTimeMs = 4500;
                if ((now - action_start_time) >= kStartIgnoreTimeMs) {
                    transitionToPool = true;
                }
                break;
            }
        }
    }

private:
    uint8_t action_stage = 0;
    uint32_t action_start_time = 0;
    bool lastLimitPressed = false;
};