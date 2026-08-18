/*
* @author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.3- Start State 
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

enum class PoolSubState { FORWARD, AVOID_LEFT, AVOID_RIGHT };
 
class PoolState {
public:
    void begin() {
        currentSubState = PoolSubState::FORWARD;
        clearStartMs = 0;
        noObstacleStartMs = 0;
        sideDetectStartMs = 0;
        poolStateStartMs = millis();
    }
 
    void update(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected, bool& transitionToLookForLine) {
        transitionToLookForLine = false;
 
        vision.stop();
        vision.clearErrors();
 
        static constexpr float kNormalSpeed = 50.0f;
        static constexpr float kAvoidSpeed = 55.0f;
        static constexpr uint32_t kNoObstacleToCornerMs = 3000;
        static constexpr uint32_t kClearDelayMs = 1500;
 
        switch (currentSubState) {
            case PoolSubState::FORWARD:
                handleForward(now, obstacle, leftDetected, rightDetected, kNormalSpeed, kNoObstacleToCornerMs, transitionToLookForLine);
                break;
 
            case PoolSubState::AVOID_LEFT:
                handleAvoidLeft(now, obstacle, leftDetected, kAvoidSpeed, kClearDelayMs);
                break;
 
            case PoolSubState::AVOID_RIGHT:
                handleAvoidRight(now, obstacle, rightDetected, kAvoidSpeed, kClearDelayMs);
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
 
    void handleForward(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected,
                       float speed, uint32_t cornerTimeout, bool& transitionToLookForLine) {
        static bool lineCorrectionActive = false;
        static uint32_t lineCorrectionStartMs = 0;
        static int8_t lineCorrectionDir = 0;
        static constexpr uint32_t kLineCorrectionMs = 120;
 
        if (lineCorrectionActive) {
            if ((now - lineCorrectionStartMs) < kLineCorrectionMs) {
                if (lineCorrectionDir < 0)
                    odomMove_.left(speed);
                else
                    odomMove_.right(speed);
                return;
            }
            lineCorrectionActive = false;
            lineCorrectionStartMs = 0;
            lineCorrectionDir = 0;
        }
 
        if (obstacle) {
            noObstacleStartMs = 0;
            setSubState(PoolSubState::AVOID_LEFT);
            return;
        }
 
        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;
 
        if (rightDetected) {
            lineCorrectionActive = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir = -1;
            odomMove_.left(speed);
            return;
        } else if (leftDetected) {
            lineCorrectionActive = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir = +1;
            odomMove_.right(speed);
            return;
        }
 
        odomMove_.forward(speed);
 
        if ((now - noObstacleStartMs) >= cornerTimeout) {
            transitionToLookForLine = true;
        }
    }
 
    void handleAvoidLeft(uint32_t now, bool obstacle, bool leftDetected,
                         float speed, uint32_t clearDelay) {
        // Checklist de seguridad: si está muy cerca, retroceder
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid() && distL < 12.0f) ||
                              (tofRight.isValid() && distR < 12.0f);
 
        if (tooClose) {
            odomMove_.backward(53.0f);
            return;
        }
 
        odomMove_.left(speed);
 
        const bool justEntered = (now - poolStateStartMs) < 150;
 
        if (leftDetected && !justEntered) {
            setSubState(PoolSubState::AVOID_RIGHT);
        } else {
            sideDetectStartMs = 0;
 
            if (!obstacle) {
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
 
    void handleAvoidRight(uint32_t now, bool obstacle, bool rightDetected,
                          float speed, uint32_t clearDelay) {
        // Checklist de seguridad
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid() && distL < 10.0f) ||
                              (tofRight.isValid() && distR < 10.0f);
 
        if (tooClose) {
            odomMove_.backward(55.0f);
            return;
        }
 
        odomMove_.right(speed);
 
        const bool justEntered = (now - poolStateStartMs) < 150;
 
        if (rightDetected && !justEntered) {
            setSubState(PoolSubState::AVOID_LEFT);
        } else {
            sideDetectStartMs = 0;
 
            if (!obstacle) {
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
 
