/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.2- Start State 
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "robot/StateMachine/States/TierOne/PoolState.hpp"

class PoolsGoBackState {
public:
    void begin() {
        currentSubState = PoolSubState::FORWARD;
        clearStartMs = 0;
        noObstacleStartMs = 0;
        sideDetectStartMs = 0;
        poolStateStartMs = millis();
    }

    void update(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected, bool& transitionToLookForLineBackwards) {
        transitionToLookForLineBackwards = false;

        static constexpr float kVelocity = 0.48f;
        static constexpr uint32_t kNoObstacleToCornerMs = 3000;
        static constexpr uint32_t kClearDelayMs = 1500;
        static constexpr uint32_t kMinAvoidTimeMs = 250;
        static constexpr uint32_t kSideDetectHoldMs = 80;

        switch (currentSubState) {
            case PoolSubState::FORWARD:
                handleForward(now, rearObstacle, leftDetected, rightDetected, kVelocity, kNoObstacleToCornerMs, transitionToLookForLineBackwards);
                break;

            case PoolSubState::AVOID_LEFT:
                handleAvoidLeft(now, rearObstacle, leftDetected, kVelocity, kClearDelayMs, kMinAvoidTimeMs, kSideDetectHoldMs);
                break;

            case PoolSubState::AVOID_RIGHT:
                handleAvoidRight(now, rearObstacle, rightDetected, kVelocity, kClearDelayMs, kMinAvoidTimeMs, kSideDetectHoldMs);
                break;
        }
    }

    PoolSubState getSubState() const { return currentSubState; }

    void setSubState(PoolSubState newState) {
        if (currentSubState == newState) return;
        currentSubState = newState;
        clearStartMs = 0;
        sideDetectStartMs = 0;
        noObstacleStartMs = 0;
        poolStateStartMs = millis();
    }

private:
    PoolSubState currentSubState = PoolSubState::FORWARD;
    uint32_t clearStartMs = 0;
    uint32_t noObstacleStartMs = 0;
    uint32_t sideDetectStartMs = 0;
    uint32_t poolStateStartMs = 0;

    void handleForward(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected,
                       float velocity, uint32_t cornerTimeout, bool& transitionToLookForLineBackwards) {
        if (rearObstacle) {
            noObstacleStartMs = 0;
            setSubState(PoolSubState::AVOID_LEFT);
        } else {
            LARC.backward(velocity);

            if (noObstacleStartMs == 0)
                noObstacleStartMs = now;

            if ((now - noObstacleStartMs) >= cornerTimeout) {
                transitionToLookForLineBackwards = true;
            }
        }
    }

    void handleAvoidLeft(uint32_t now, bool rearObstacle, bool leftDetected,
                         float velocity, uint32_t clearDelay, uint32_t minAvoidMs, uint32_t sideDetectMs) {
        LARC.left(velocity);

        const bool canChangeSide = (now - poolStateStartMs) >= minAvoidMs;

        if (leftDetected) {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= sideDetectMs) {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setSubState(PoolSubState::AVOID_RIGHT);
            }
        } else {
            sideDetectStartMs = 0;

            if (!rearObstacle) {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= clearDelay) {
                    noObstacleStartMs = 0;
                    setSubState(PoolSubState::FORWARD);
                }
            } else {
                clearStartMs = 0;
            }
        }
    }

    void handleAvoidRight(uint32_t now, bool rearObstacle, bool rightDetected,
                          float velocity, uint32_t clearDelay, uint32_t minAvoidMs, uint32_t sideDetectMs) {
        LARC.right(velocity);

        const bool canChangeSide = (now - poolStateStartMs) >= minAvoidMs;

        if (rightDetected) {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= sideDetectMs) {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setSubState(PoolSubState::AVOID_LEFT);
            }
        } else {
            sideDetectStartMs = 0;

            if (!rearObstacle) {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= clearDelay) {
                    noObstacleStartMs = 0;
                    setSubState(PoolSubState::FORWARD);
                }
            } else {
                clearStartMs = 0;
            }
        }
    }
};