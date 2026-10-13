/*
* @author:  Ximena Patricia García Magdaleno
* PoolState.hpp
* State Machine Tier One.2- Pool State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

enum class PoolSubState { FORWARD, AVOID_LEFT, AVOID_RIGHT };

class PoolState {
public:
    void begin() {
        currentSubState = PoolSubState::FORWARD;
        clearStartMs = 0;
        noObstacleStartMs = 0;
        sideDetectStartMs = 0;
        poolStateStartMs = 0;
    }

    void update(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected,
                bool tofReady, bool& transitionToLookForLine) {
        transitionToLookForLine = false;

        vision.stop();
        vision.clearErrors();

        switch (currentSubState) {
            case PoolSubState::FORWARD:
                handleForward(now, obstacle, leftDetected, rightDetected, tofReady, transitionToLookForLine);
                break;

            case PoolSubState::AVOID_LEFT:
                handleAvoidLeft(now, obstacle, leftDetected, rightDetected);
                break;

            case PoolSubState::AVOID_RIGHT:
                handleAvoidRight(now, obstacle, leftDetected, rightDetected);
                break;
        }
    }

    PoolSubState getSubState() const { return currentSubState; }

    static const __FlashStringHelper *subStateName(PoolSubState state) {
        switch (state) {
            case PoolSubState::FORWARD:     return F(""); // F("FORWARD");
            case PoolSubState::AVOID_LEFT:  return F(""); // F("AVOID_LEFT");
            case PoolSubState::AVOID_RIGHT: return F(""); // F("AVOID_RIGHT");
            default:                        return F(""); // F("DEFAULT");
        }
    }

private:
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

        // Serial.print(F("Pool substate -> "));
        // Serial.println(subStateName(currentSubState));
    }

    void handleForward(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected,
                       bool tofReady, bool& transitionToLookForLine) {
        using namespace StateCommon;

        static bool     lineCorrectionActive   = false;
        static uint32_t lineCorrectionStartMs  = 0;
        static int8_t   lineCorrectionDir      = 0;

        if (lineCorrectionActive) {
            if ((now - lineCorrectionStartMs) < Constants::StateMachine::Pool::kLineCorrectionMs) {
                if (lineCorrectionDir < 0)
                    LARC.left(kManeuverSpeed);
                else
                    LARC.right(kManeuverSpeed);
                return;
            }
            lineCorrectionActive  = false;
            lineCorrectionStartMs = 0;
            lineCorrectionDir     = 0;
        }

        if (!tofReady) {
            noObstacleStartMs = 0;
            LARC.stop();
            return;
        }

        if (obstacle) {
            noObstacleStartMs = 0;
            setSubState(PoolSubState::AVOID_LEFT);
            return;
        }

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if (rightDetected) {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = -1;
            LARC.left(kManeuverSpeed);
            return;
        } else if (leftDetected) {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = +1;
            LARC.right(kManeuverSpeed);
            return;
        }

        LARC.forward(kManeuverSpeed);

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs) {
            transitionToLookForLine = true;
        }
    }

    void handleAvoidLeft(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected) {
        using namespace StateCommon;

        // Si el obstáculo se acerca demasiado, retroceder
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidLeftMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidLeftMm);

        if (tooClose) {
            LARC.backward(kManeuverSpeed);
            return;
        }

        LARC.left(kManeuverSpeed);

        const bool justEntered = (now - poolStateStartMs) < Constants::StateMachine::Pool::kJustEnteredLeftMs;
        // Both sides at once means a line crossing the robot, not the left border.
        const bool borderReached = leftDetected && !rightDetected && !justEntered;

        if (handleBorder(now, obstacle, borderReached, PoolSubState::AVOID_RIGHT))
            return;

        updateClear(now, obstacle);
    }

    void handleAvoidRight(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected) {
        using namespace StateCommon;

        // Si el obstáculo se acerca demasiado, retroceder
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidRightMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidRightMm);

        if (tooClose) {
            LARC.backward(kManeuverSpeed);
            return;
        }

        LARC.right(kManeuverSpeed);

        const bool justEntered = (now - poolStateStartMs) < Constants::StateMachine::Pool::kJustEnteredRightMs;
        const bool borderReached = rightDetected && !leftDetected && !justEntered;

        if (handleBorder(now, obstacle, borderReached, PoolSubState::AVOID_LEFT))
            return;

        updateClear(now, obstacle);
    }

    // Border line held for kSideDetectHoldMs: if the obstacle is already clear,
    // go FORWARD instead of strafing back in front of it.
    bool handleBorder(uint32_t now, bool obstacle, bool borderReached, PoolSubState opposite) {
        using namespace StateCommon;

        if (!borderReached) {
            sideDetectStartMs = 0;
            return false;
        }

        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs) {
            noObstacleStartMs = 0;
            setSubState(obstacle ? opposite : PoolSubState::FORWARD);
        }
        return true;
    }

    void updateClear(uint32_t now, bool obstacle) {
        using namespace StateCommon;

        if (obstacle) {
            clearStartMs = 0;
            return;
        }

        if (clearStartMs == 0)
            clearStartMs = now;

        if ((now - clearStartMs) >= kClearDelayMs) {
            noObstacleStartMs = 0;
            setSubState(PoolSubState::FORWARD);
        }
    }
};
