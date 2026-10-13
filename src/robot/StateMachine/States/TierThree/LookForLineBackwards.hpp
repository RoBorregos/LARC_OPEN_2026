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
#include "robot/StateMachine/States/IrSideLatch.hpp"

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
        namespace cfg = Constants::StateMachine::LookForLineBackwards;
        transitionToBenefitsStartCorner = false;

        if (tracker_.update() == QtrEntryTracker::Phase::ON) {
            correcting_ = false;
            Serial.println("[LOOKFORLINEBACKWARDS] QTR ON -> BENEFITSSTARTCORNER");
            LARC.stop();
            transitionToBenefitsStartCorner = true;
            return;
        }

        if (correcting_) {
            if ((now - correctionStart_) < cfg::kBorderCorrectMs) {
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
    static constexpr float kSpeed = Constants::StateMachine::Common::kManeuverSpeed;

    // Floor already reads norm ~470 on qtrRear, so thresholds sit above it.
    static QtrEntryTracker::Config trackerCfg() {
        namespace cfg = Constants::StateMachine::LookForLineBackwards;
        return QtrEntryTracker::makeConfig(cfg::kEntryAtHighIndex, cfg::kCenterProgress);
    }

    QtrEntryTracker tracker_{qtrRear, trackerCfg()};
    IrSideLatch left_;
    IrSideLatch right_;

    bool     correcting_      = false;
    int8_t   correctionDir_   = 0;
    uint32_t correctionStart_ = 0;

    void startCorrection(uint32_t now, int8_t dir) {
        correcting_      = true;
        correctionDir_   = dir;
        correctionStart_ = now;
    }
};
