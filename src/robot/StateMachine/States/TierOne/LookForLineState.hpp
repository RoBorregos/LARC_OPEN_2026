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
    }

    // Lateral IR pairs: left side = FL + BL, right side = FR + BR.
    void update(uint32_t now, bool FL, bool FR, bool BL, bool BR, bool& transitionToCorner) {
        transitionToCorner = false;

        // ── Stage 0: retroceder 200 ms ────────────────────────────────────────
        if (action_stage == 0) {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(0.30f);

            if ((now - action_start_time) >= 200) {
                action_stage = 1;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 1: retroceder 200 ms más ────────────────────────────────────
        if (action_stage == 1) {
            LARC.backward(0.30f);

            if ((now - action_start_time) >= 200) {
                action_stage = 2;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 2: avanzar 300 ms ───────────────────────────────────────────
        if (action_stage == 2) {
            LARC.forward(0.30f);

            if ((now - action_start_time) >= 300) {
                action_stage = 3;
                action_start_time = now;
            }
            return;
        }

        // ── Stage 3: búsqueda normal ───────────────────────────────────────────
        if (tracker_.update() == QtrEntryTracker::Phase::ON) {
            correcting_ = false;
            Serial.println("[LOOKFORLINE] QTR ON -> LOOKFORCORNER");
            LARC.stop();
            transitionToCorner = true;
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

        LARC.forward(kSpeed);
    }

private:
    static constexpr float    kSpeed           = 0.30f;
    static constexpr uint32_t kBorderCorrectMs = 150;

    // qtrFront: C0..C6 (indices 0..6), C6 is the frontmost sensor.
    // Set to false if the line enters through C0 when going forward.
    static constexpr bool kEntryAtHighIndex = true;

    static QtrEntryTracker::Config trackerCfg() {
        return {
            /*entryAtHighIndex*/ kEntryAtHighIndex,
            /*onThreshold*/      Constants::QTRCalibration::kBinaryThreshold,
            /*offThreshold*/     Constants::QTRCalibration::kBinaryThreshold - 80,
            /*entryMaxProgress*/ 1.0f,
            /*centerProgress*/   3.0f,
            /*centerTolerance*/  0.75f,
            /*maxRegress*/       0.6f,
            /*confirmReads*/     2,
            /*onReads*/          3,
            /*lostReads*/        5,
        };
    }

    int action_stage = 0;
    uint32_t action_start_time = 0;

    QtrEntryTracker tracker_{qtrFront, trackerCfg()};
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
