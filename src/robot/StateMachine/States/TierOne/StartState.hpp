/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.1- Start State 
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

//@note: Refactor StartState to solve redundancy in elevator states, when its ready to use

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

        //Elevator
        if (limitPressed != lastLimitPressed) {
            if (limitPressed)
                Serial.println("LIMIT SWITCH PRESIONADO");
            else
                Serial.println("LIMIT SWITCH LIBERADO");
            lastLimitPressed = limitPressed;
        }

        switch (action_stage) {
            // ── Stage 0: Goes up for 12000 ms ────────────────────────────────────
            case 0: {
                if (limitPressed) {
                    // If Limit switch is pressed during the ascent - interrupt and descend
                    elevator.ElevatorPosition(0);
                    odomMove_.stop();
                    action_start_time = now;
                    action_stage = 1;
                } else {
                    elevator.ElevatorPosition(2);
                    odomMove_.stop();

                    if ((now - action_start_time) >= 12000) {
                        // Finishes going up - elevetor goes to "stop" state
                        action_start_time = now;
                        action_stage = 4;
                    }
                }
                break;
            }

            // ── Stage 1: Elevator goes down while limit switch is pressed ───────────────────
            case 1: {
                elevator.ElevatorPosition(1);
                odomMove_.stop();

                if (!limitPressed) {
                    // Limit switch release - waits 2000 ms before going up again
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            // ── Stage 2: Waits 2000 ms with the elevator being stopped──────────────────
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