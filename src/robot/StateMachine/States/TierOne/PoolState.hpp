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
                handleAvoidLeft(now, obstacle, leftDetected);
                break;

            case PoolSubState::AVOID_RIGHT:
                handleAvoidRight(now, obstacle, rightDetected);
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

        Serial.print(F("Pool substate -> "));
        Serial.println(subStateName(currentSubState));
    }

    void handleForward(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected,
                       bool tofReady, bool& transitionToLookForLine) {
        using namespace StateCommon;

        static bool     lineCorrectionActive   = false;
        static uint32_t lineCorrectionStartMs  = 0;
        static int8_t   lineCorrectionDir      = 0;

        static constexpr uint32_t kLineCorrectionMs = 120;
        static constexpr float    kNormalSpeed = 0.30f;

        if (lineCorrectionActive) {
            if ((now - lineCorrectionStartMs) < kLineCorrectionMs) {
                if (lineCorrectionDir < 0)
                    LARC.left(kNormalSpeed);
                else
                    LARC.right(kNormalSpeed);
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
            LARC.left(kNormalSpeed);
            return;
        } else if (leftDetected) {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = +1;
            LARC.right(kNormalSpeed);
            return;
        }

        LARC.forward(kNormalSpeed);

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs) {
            transitionToLookForLine = true;
        }
    }

    void handleAvoidLeft(uint32_t now, bool obstacle, bool leftDetected) {
        using namespace StateCommon;

        // Si el obstáculo se acerca demasiado, retroceder
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidLeftMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidLeftMm);

        if (tooClose) {
            LARC.backward(0.30f);
            return;
        }

        LARC.left(0.30f);

        const bool justEntered = (now - poolStateStartMs) < 150;

        if (leftDetected && !justEntered) {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setSubState(PoolSubState::AVOID_RIGHT);
        } else {
            sideDetectStartMs = 0;

            if (!obstacle) {
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

    void handleAvoidRight(uint32_t now, bool obstacle, bool rightDetected) {
        using namespace StateCommon;

        // Si el obstáculo se acerca demasiado, retroceder
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidRightMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidRightMm);

        if (tooClose) {
            LARC.backward(0.30f);
            return;
        }

        LARC.right(0.30f);

        const bool justEntered = (now - poolStateStartMs) < 100;

        if (rightDetected && !justEntered) {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setSubState(PoolSubState::AVOID_LEFT);
        } else {
            sideDetectStartMs = 0;

            if (!obstacle) {
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
