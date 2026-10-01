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
            case 0: {
                if (limitPressed) {
                    elevator.ElevatorPosition(0);
                    odomMove_.stop();
                    action_start_time = now;
                    action_stage = 1;
                } else {
                    elevator.ElevatorPosition(2);
                    odomMove_.stop();

                    if ((now - action_start_time) >= 12000) {
                        action_start_time = now;
                        action_stage = 4;
                    }
                }
                break;
            }

            case 1: {
                elevator.ElevatorPosition(1);
                odomMove_.stop();

                if (!limitPressed) {
                    action_start_time = now;
                    action_stage = 2;
                }
                break;
            }

            case 2: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();

                if ((now - action_start_time) >= 2000) {
                    action_start_time = now;
                    action_stage = 0;
                }
                break;
            }

            case 4: {
                elevator.ElevatorPosition(0);
                odomMove_.stop();
                if ((now - action_start_time) >= 1500) {
                    action_start_time = now;
                    action_stage = 5;
                }
                break;
            }

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