/*
*@author:  Ximena Patricia García Magdaleno
* LookForLineState.hpp
* State Machine Tier One.3- Look For Line State
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"
#include "robot/StateMachine/States/QtrEntryTracker.hpp"
#include "robot/StateMachine/States/IrSideLatch.hpp"

class LookForLineState {
public:
    void begin() {
        action_stage = 0;
        action_start_time = 0;
        tracker_.reset();
        left_.clear();
        right_.clear();
        correcting_      = false;
        correctionDir_   = 0;
        correctionStart_ = 0;
        lastFrontIrMs_   = 0;
        qtrOnMs_         = 0;
    }

    // Lateral IR pairs: left side = FL + BL, right side = FR + BR.
    void update(uint32_t now, bool FL, bool FR, bool BL, bool BR, bool& transitionToCorner) {
        namespace cfg = Constants::StateMachine::LookForLine;
        transitionToCorner = false;

        // ── Stage 0: retroceder 200 ms ────────────────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(kSpeed);

            if ((now - action_start_time) >= cfg::kBackupMs) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 1: retroceder 200 ms más ────────────────────────────────────
        if (action_stage == 1) {
            LARC.backward(kSpeed);

            if ((now - action_start_time) >= cfg::kBackupMs) {
                action_stage = 2;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 2: avanzar 300 ms ───────────────────────────────────────────
        if (action_stage == 2) {
            LARC.forward(kSpeed);

            if ((now - action_start_time) >= cfg::kForwardMs) {
                action_stage = 3;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 3: búsqueda normal ───────────────────────────────────────────
        // QTR detects the line, a front IR (FL/FR) confirms it within kIrConfirmMs.
        // Dark patches that only the QTR sees are discarded.
        if (FL || FR)
            lastFrontIrMs_ = now;
        const bool irRecent = lastFrontIrMs_ != 0 && (now - lastFrontIrMs_) <= cfg::kIrConfirmMs;

        if (tracker_.update() == QtrEntryTracker::Phase::ON) {
            if (qtrOnMs_ == 0)
                qtrOnMs_ = now;

            if (irRecent) {
                correcting_ = false;
                // Serial.println("[LOOKFORLINE] QTR ON + IR -> LOOKFORCORNER");
                LARC.stop();
                transitionToCorner = true;
                return;
            }
            if ((now - qtrOnMs_) > cfg::kIrConfirmMs) {
                // Serial.println("[LOOKFORLINE] QTR ON without IR -> ignored");
                tracker_.reset();
                qtrOnMs_ = 0;
            }
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

        LARC.forward(kSpeed);
    }

private:
    static constexpr float kSpeed = Constants::StateMachine::Common::kManeuverSpeed;

    static QtrEntryTracker::Config trackerCfg() {
        namespace cfg = Constants::StateMachine::LookForLine;
        return QtrEntryTracker::makeConfig(cfg::kEntryAtHighIndex, cfg::kCenterProgress);
    }

    int action_stage = 0;
    uint32_t action_start_time = 0;

    QtrEntryTracker tracker_{qtrFront, trackerCfg()};
    IrSideLatch left_;
    IrSideLatch right_;

    bool     correcting_      = false;
    int8_t   correctionDir_   = 0;
    uint32_t correctionStart_ = 0;
    uint32_t lastFrontIrMs_   = 0;
    uint32_t qtrOnMs_         = 0;

    void startCorrection(uint32_t now, int8_t dir) {
        correcting_      = true;
        correctionDir_   = dir;
        correctionStart_ = now;
    }
};
