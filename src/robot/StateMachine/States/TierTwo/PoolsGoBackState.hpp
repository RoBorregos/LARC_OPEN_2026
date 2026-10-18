/*
*@author:  Ximena Patricia García Magdaleno
* PoolsGoBackState.hpp
* State Machine Tier Two.4- Pools Go Back State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"
#include "../TierOne/PoolState.hpp"

class PoolsGoBackState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        currentSubState = PoolSubState::FORWARD;
        clearStartMs = 0;
        noObstacleStartMs = 0;
        sideDetectStartMs = 0;
        poolStateStartMs = 0;
        keepRight = false;
        poolSeen = false;
    }

    void update(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected,
                bool tofReady, bool& transitionToLookForLineBackwards) {
        using namespace StateCommon;
        namespace cfg = Constants::StateMachine::PoolsGoBack;
        transitionToLookForLineBackwards = false;

        // A pool behind during the initial maneuver: start avoiding right away.
        if (action_stage < 2 && tofReady && rearObstacle) {
            action_stage = 2;
            action_start_time = 0;
            setSubState(PoolSubState::AVOID_LEFT);
        }

        // ── Stage 0: retroceder kInitBackMs ─────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(kVelocity);

            if ((now - action_start_time) >= cfg::kInitBackMs) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 1: izquierda kInitLeftMs ──────────────────────────────
        if (action_stage == 1) {
            LARC.left(kVelocity);

            if ((now - action_start_time) >= cfg::kInitLeftMs) {
                action_stage = 2;
                action_start_time = 0;
            }
            return;
        }

        // ── Stage 2: esquivar albercas en reversa ───────────────────────
        switch (currentSubState) {
            case PoolSubState::FORWARD:
                handleForward(now, rearObstacle, tofReady, transitionToLookForLineBackwards);
                break;

            case PoolSubState::AVOID_LEFT:
                handleAvoidLeft(now, rearObstacle, leftDetected);
                break;

            case PoolSubState::AVOID_RIGHT:
                handleAvoidRight(now, rearObstacle, rightDetected);
                break;
        }
    }

    PoolSubState getSubState() const { return currentSubState; }

private:
    int action_stage = 0;
    uint32_t action_start_time = 0;

    PoolSubState currentSubState = PoolSubState::FORWARD;
    uint32_t clearStartMs = 0;
    uint32_t noObstacleStartMs = 0;
    uint32_t sideDetectStartMs = 0;
    uint32_t poolStateStartMs = 0;
    bool     keepRight = false; // left border already hit: keep avoiding right until the obstacle is passed
    bool     poolSeen = false; // a pool was avoided in this state

    void setSubState(PoolSubState newState) {
        if (currentSubState == newState) return;
        if (currentSubState == PoolSubState::AVOID_LEFT && newState == PoolSubState::AVOID_RIGHT)
            keepRight = true;
        else if (currentSubState == PoolSubState::AVOID_RIGHT && newState == PoolSubState::AVOID_LEFT)
            keepRight = false;
        currentSubState = newState;
        clearStartMs = 0;
        sideDetectStartMs = 0;
        noObstacleStartMs = 0;
        poolStateStartMs = millis();
        if (newState != PoolSubState::FORWARD)
            poolSeen = true;

        // Serial.print(F("Pool substate -> "));
        // Serial.println(PoolState::subStateName(currentSubState));
    }

    void handleForward(uint32_t now, bool rearObstacle, bool tofReady,
                       bool& transitionToLookForLineBackwards) {
        using namespace StateCommon;

        if (!tofReady) {
            noObstacleStartMs = 0;
            LARC.stop();
            return;
        }

        if (rearObstacle) {
            noObstacleStartMs = 0;
            setSubState(keepRight ? PoolSubState::AVOID_RIGHT : PoolSubState::AVOID_LEFT);
            return;
        }

        LARC.backward(kVelocity);

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kKeepRightResetMs)
            keepRight = false;

        const uint32_t exitMs = poolSeen ? kNoObstacleToCornerMs
                                         : Constants::StateMachine::PoolsGoBack::kFirstPoolTimeoutMs;
        if ((now - noObstacleStartMs) >= exitMs) {
            transitionToLookForLineBackwards = true;
        }
    }

    void handleAvoidLeft(uint32_t now, bool rearObstacle, bool leftDetected) {
        using namespace StateCommon;

        const bool tooCloseRear = (tofBackLeft.isValid()  && tofBackLeft.getDistanceMm()  < kTooCloseAvoidLeftMm) ||
                                  (tofBackRight.isValid() && tofBackRight.getDistanceMm() < kTooCloseAvoidLeftMm);
        if (tooCloseRear) {
            LARC.forward(kManeuverSpeed);
            return;
        }

        LARC.left(kManeuverSpeed);

        if (leftDetected) {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= kSideDetectHoldMs) {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setSubState(PoolSubState::AVOID_RIGHT);
            }
        } else {
            sideDetectStartMs = 0;

            if (!rearObstacle) {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs) {
                    noObstacleStartMs = 0;
                    setSubState(PoolSubState::FORWARD);
                }
            } else {
                clearStartMs = 0;
            }
        }
    }

    void handleAvoidRight(uint32_t now, bool rearObstacle, bool rightDetected) {
        using namespace StateCommon;

        const bool tooCloseRear = (tofBackLeft.isValid()  && tofBackLeft.getDistanceMm()  < kTooCloseAvoidRightMm) ||
                                  (tofBackRight.isValid() && tofBackRight.getDistanceMm() < kTooCloseAvoidRightMm);
        if (tooCloseRear) {
            LARC.forward(kManeuverSpeed);
            return;
        }

        LARC.right(kManeuverSpeed);

        if (rightDetected) {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= kSideDetectHoldMs) {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setSubState(PoolSubState::AVOID_LEFT);
            }
        } else {
            sideDetectStartMs = 0;

            if (!rearObstacle) {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs) {
                    noObstacleStartMs = 0;
                    setSubState(PoolSubState::FORWARD);
                }
            } else {
                clearStartMs = 0;
            }
        }
    }
};
