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
#include "robot/StateMachine/States/StateCommon.hpp"

class LookForLineBackwardsState {
public:
    void begin() {
        tracker_.reset();
        left_.clear();
        right_.clear();
        correcting_      = false;
        correctionDir_   = 0;
        correctionStart_ = 0;
        onLineSinceMs_   = 0;
        slow_            = false;
    }

    // Lateral IR pairs: left side = FL + BL, right side = FR + BR.
    // rearLeftMm/rearRightMm: LL/LR distance (0 = no reading).
    void update(uint32_t now, bool FL, bool FR, bool BL, bool BR,
                uint16_t rearLeftMm, uint16_t rearRightMm,
                bool& transitionToBenefitsStartCorner) {
        namespace cfg = Constants::StateMachine::LookForLineBackwards;
        transitionToBenefitsStartCorner = false;

        // Latched: right at the box the ToF stops measuring and reads "nothing there".
        bool atBoxes = false;
        for (const uint16_t d : {rearLeftMm, rearRightMm}) {
            if (d != 0 && d <= cfg::kSlowDistMm)
                slow_ = true;
            if (d != 0 && d <= cfg::kStopDistMm)
                atBoxes = true;
        }

        // Plain qtrRear onLine (kRearOnLineThreshold), held kOnLineConfirmMs to skip noise spikes
        if (StateCommon::rearOnLine()) {
            if (onLineSinceMs_ == 0)
                onLineSinceMs_ = now;
        } else {
            onLineSinceMs_ = 0;
        }
        const bool onLineConfirmed = onLineSinceMs_ != 0 && (now - onLineSinceMs_) >= cfg::kOnLineConfirmMs;

        // At the boxes before the line: stop here, BENEFITS fixes the distance with the ToFs.
        if (tracker_.update() == QtrEntryTracker::Phase::ON || onLineConfirmed || atBoxes) {
            correcting_ = false;
            // Serial.println("[LOOKFORLINEBACKWARDS] QTR ON -> BENEFITSSTARTCORNER");
            // stop() alone ramps down and coasts into the boxes; brake() alone keeps the old
            // backward command, so the next control tick would drive backward again.
            LARC.stop();
            LARC.brake();
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

        LARC.backward(slow_ ? cfg::kSlowSpeed : kSpeed);
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
    uint32_t onLineSinceMs_   = 0;
    bool     slow_            = false; // a rear ToF saw the boxes within kSlowDistMm

    void startCorrection(uint32_t now, int8_t dir) {
        correcting_      = true;
        correctionDir_   = dir;
        correctionStart_ = now;
    }
};
