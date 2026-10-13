/*
*@author: Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.1- Start State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"
#include "../YawSettle.hpp"

class StartState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = millis();
        elevatorCmd_ = -1;
        // The elevator can brown out the BNO085 and its yaw drifts for a while after:
        // no yaw hold until the robot is about to drive, so it cannot rotate in place.
        LARC.holdYaw(false);
    }

    void update(uint32_t now, bool obstacle, bool tofReady, bool& transitionToPool) {
        using namespace StateCommon;
        namespace cfg = Constants::StateMachine::Start;
        transitionToPool = false;

        vision.stop();

        const bool limitPressed = kLimitSwitchConnected && (digitalRead(Pins::kLimitSwitch) == kLimitPressedLevel);

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

                    if ((now - action_start_time) >= cfg::kElevatorUpMs) {
                        // Finishes going up - elevetor goes to "stop" state
                        action_start_time = now;
                        yawSettle_.reset(now, LARC.getYaw());
                        action_stage = 4;
                    }
                }
                break;

            // ── Stage 4: Elevador stop, wait for a stable yaw, then latch it ──
            case 4:
                setElevator(kElevatorStop);
                LARC.stop();
                if (yawSettle_.update(now, LARC.getYaw(), cfg::kElevatorStopMs)) {
                    LARC.holdYaw(true);
                    action_start_time = now;
                    action_stage = 5;
                }
                break;

            // ── Stage 5: Avanzar y transicionar a POOL ───────────────────────
            // IRs are ignored for kStartIgnoreTimeMs (start square lines), the ToFs are not:
            // a pool ahead goes straight to POOL.
            case 5:
                setElevator(kElevatorStop);
                LARC.forward(cfg::kForwardSpeed);

                if ((tofReady && obstacle) || (now - action_start_time) >= kStartIgnoreTimeMs) {
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
    YawSettle yawSettle_;

    void setElevator(int cmd) {
        using namespace StateCommon;
        if (cmd == elevatorCmd_)
            return;
        elevatorCmd_ = cmd;
        elevator.ElevatorPosition(cmd);
        //Serial.println(cmd == kElevatorUp ? "[elevator] UP" : (cmd == kElevatorDown ? "[elevator] DOWN" : "[elevator] STOP"));
    }
};
