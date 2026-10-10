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
    }

    void update(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected,
                bool tofReady, bool& transitionToLookForLineBackwards) {
        using namespace StateCommon;
        transitionToLookForLineBackwards = false;

        static constexpr uint32_t kInitBackMs = 500;
        static constexpr uint32_t kInitLeftMs = 500;

        // ── Stage 0: retroceder kInitBackMs ─────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(kVelocity);

            if ((now - action_start_time) >= kInitBackMs) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 1: izquierda kInitLeftMs ──────────────────────────────
        if (action_stage == 1) {
            LARC.left(kVelocity);

            if ((now - action_start_time) >= kInitLeftMs) {
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

    void setSubState(PoolSubState newState) {
        if (currentSubState == newState) return;
        currentSubState = newState;
        clearStartMs = 0;
        sideDetectStartMs = 0;
        noObstacleStartMs = 0;
        poolStateStartMs = millis();

        Serial.print(F("Pool substate -> "));
        Serial.println(PoolState::subStateName(currentSubState));
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
            setSubState(PoolSubState::AVOID_LEFT);
            return;
        }

        LARC.backward(kVelocity);

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs) {
            transitionToLookForLineBackwards = true;
        }
    }

    void handleAvoidLeft(uint32_t now, bool rearObstacle, bool leftDetected) {
        using namespace StateCommon;

        const bool tooCloseRear = (tofBackLeft.isValid()  && tofBackLeft.getDistanceMm()  < kTooCloseAvoidLeftMm) ||
                                  (tofBackRight.isValid() && tofBackRight.getDistanceMm() < kTooCloseAvoidLeftMm);
        if (tooCloseRear) {
            LARC.forward(0.30f);
            return;
        }

        LARC.left(0.30f);

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
            LARC.forward(0.30f);
            return;
        }

        LARC.right(0.30f);

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
