/*
*@author:  Ximena Patricia García Magdaleno
* LookForLineBackwards.hpp
* State Machine Tier Three.1- Look For Line Backwards State
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"
#include "robot/StateMachine/States/QtrEntryTracker.hpp"

class LookForLineBackwardsState {
public:
    void begin() {
        tracker_.reset();
        left_.clear();
        right_.clear();
        correcting_      = false;
        correctionDir_   = 0;
        correctionStart_ = 0;
    }

    // Lateral IR pairs: left side = FL + BL, right side = FR + BR.
    void update(uint32_t now, bool FL, bool FR, bool BL, bool BR,
                bool& transitionToBenefitsStartCorner) {
        transitionToBenefitsStartCorner = false;

        if (tracker_.update() == QtrEntryTracker::Phase::ON) {
            correcting_ = false;
            Serial.println("[LOOKFORLINEBACKWARDS] QTR ON -> BENEFITSSTARTCORNER");
            LARC.stop();
            transitionToBenefitsStartCorner = true;
            return;
        }

        if (correcting_) {
            if ((now - correctionStart_) < kBorderCorrectMs) {
                if (correctionDir_ < 0)
                    LARC.left(kSpeed);
                else
                    LARC.right(kSpeed);
                return;
            }
            correcting_ = false;
        }

        const bool leftCorrect  = left_.update(now, FL, BL);
        const bool rightCorrect = right_.update(now, FR, BR);

        // Both sides at once means a line crossing the robot, not a side border.
        if (left_.latched && right_.latched) {
            left_.clear();
            right_.clear();
        } else if (leftCorrect && !(FR || BR)) {
            startCorrection(now, +1);
            LARC.right(kSpeed);
            return;
        } else if (rightCorrect && !(FL || BL)) {
            startCorrection(now, -1);
            LARC.left(kSpeed);
            return;
        }

        LARC.backward(kSpeed);
    }

private:
    static constexpr float    kSpeed           = 0.30f;
    static constexpr uint32_t kBorderCorrectMs = 150;
    static constexpr uint32_t kLatchTimeoutMs  = 800;
    static constexpr uint32_t kLatchIrOffMs    = 150;

    // One side's IR pair: first IR latches, the other IR of the same side triggers the correction.
    struct SideLatch {
        bool     latched   = false;
        bool     firstFront = false;
        uint32_t latchMs   = 0;
        uint32_t offSinceMs = 0;

        void clear() {
            latched    = false;
            latchMs    = 0;
            offSinceMs = 0;
        }

        bool update(uint32_t now, bool front, bool back) {
            if (!latched) {
                if (front && back)
                    return true;
                if (front || back) {
                    latched    = true;
                    firstFront = front;
                    latchMs    = now;
                    offSinceMs = 0;
                }
                return false;
            }

            const bool firstOn = firstFront ? front : back;
            const bool otherOn = firstFront ? back : front;

            if (otherOn) {
                clear();
                return true;
            }
            if ((now - latchMs) >= kLatchTimeoutMs) {
                clear();
                return false;
            }
            if (firstOn) {
                offSinceMs = 0;
            } else {
                if (offSinceMs == 0)
                    offSinceMs = now;
                if ((now - offSinceMs) >= kLatchIrOffMs)
                    clear();
            }
            return false;
        }
    };

    // qtrRear: C8..C13 (indices 0..5), the line enters through C8 when going backwards.
    // Floor already reads norm ~470 on qtrRear, so thresholds sit above it.
    static QtrEntryTracker::Config trackerCfg() {
        return {
            /*entryAtHighIndex*/ false,
            /*onThreshold*/      Constants::QTRCalibration::kBinaryThreshold,
            /*offThreshold*/     Constants::QTRCalibration::kBinaryThreshold - 80,
            /*entryMaxProgress*/ 1.0f,
            /*centerProgress*/   2.5f,
            /*centerTolerance*/  0.75f,
            /*maxRegress*/       0.6f,
            /*confirmReads*/     2,
            /*onReads*/          3,
            /*lostReads*/        5,
        };
    }

    QtrEntryTracker tracker_{qtrRear, trackerCfg()};
    SideLatch left_;
    SideLatch right_;

    bool     correcting_      = false;
    int8_t   correctionDir_   = 0;
    uint32_t correctionStart_ = 0;

    void startCorrection(uint32_t now, int8_t dir) {
        correcting_      = true;
        correctionDir_   = dir;
        correctionStart_ = now;
    }
};
