/*
*@author: Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.1- Start State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class StartState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = millis();
        elevatorCmd_ = -1;
    }

    void update(uint32_t now, bool& transitionToPool) {
        using namespace StateCommon;
        transitionToPool = false;

        vision.stop();

        const bool limitPressed = kLimitSwitchConnected && (digitalRead(Pins::kLimitSwitch) == kLimitPressedLevel);

        //Elevator
        if (limitPressed != lastLimitPressed) {
            if (limitPressed)
                Serial.println("LIMIT SWITCH PRESIONADO");
            else
                Serial.println("LIMIT SWITCH LIBERADO");
            lastLimitPressed = limitPressed;
        }

        switch (action_stage) {
            // ── Stage 0: Up by default, down while the limit switch is pressed ──
            case 0:
                LARC.stop();

                if (limitPressed) {
                    setElevator(kElevatorDown);
                    // The 12000 ms ascent restarts once the switch is released
                    action_start_time = now;
                } else {
                    setElevator(kElevatorUp);

                    if ((now - action_start_time) >= 12000) {
                        // Finishes going up - elevetor goes to "stop" state
                        action_start_time = now;
                        action_stage = 4;
                    }
                }
                break;

            // ── Stage 4: Elevador stop 1500 ms ───────────────────────────────
            case 4:
                setElevator(kElevatorStop);
                LARC.stop();
                if ((now - action_start_time) >= 1500) {
                    action_start_time = now;
                    action_stage = 5;
                }
                break;

            // ── Stage 5: Avanzar y transicionar a POOL ───────────────────────
            case 5:
                setElevator(kElevatorStop);
                LARC.forward(0.30f);

                if ((now - action_start_time) >= kStartIgnoreTimeMs) {
                    transitionToPool = true;
                }
                break;
        }
    }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;
    bool lastLimitPressed = false;
    int  elevatorCmd_ = -1;

    void setElevator(int cmd) {
        using namespace StateCommon;
        if (cmd == elevatorCmd_)
            return;
        elevatorCmd_ = cmd;
        elevator.ElevatorPosition(cmd);
        Serial.println(cmd == kElevatorUp ? "[elevator] UP" : (cmd == kElevatorDown ? "[elevator] DOWN" : "[elevator] STOP"));
    }
};
