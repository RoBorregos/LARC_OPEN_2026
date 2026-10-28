/*
* IntakeRamp.hpp
* Intake ramp started when POOL begins after START: kIntakeSteps positions,
* each held kIntakeStepMs, then home. Runs across states until it finishes.
*/
#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"

class IntakeRamp {
public:
    void start(uint32_t now) {
        servos.holdIntakes(true);
        step_ = 0;
        stepTime_ = now;
        active_ = true;
        writeStep();
    }

    void update(uint32_t now) {
        namespace cfg = Constants::StateMachine::Start;
        if (!active_ || (now - stepTime_) < cfg::kIntakeStepMs)
            return;

        if (step_ + 1 < cfg::kIntakeSteps) {
            step_++;
            stepTime_ = now;
            writeStep();
        } else {
            finish();
        }
    }

    // Straight to home (end of the ramp, or cut short).
    void finish() {
        if (!active_)
            return;
        active_ = false;
        servos.holdIntakes(false);
        servos.setIntakeUpper(false);
        servos.setIntakeLower(false);
    }

    bool active() const { return active_; }

private:
    bool     active_ = false;
    uint8_t  step_ = 0;
    uint32_t stepTime_ = 0;

    void writeStep() {
        namespace cfg = Constants::StateMachine::Start;
        servos.setIntakeAngles(cfg::kIntakeUpperSteps[step_], cfg::kIntakeLowerSteps[step_]);
    }
};
