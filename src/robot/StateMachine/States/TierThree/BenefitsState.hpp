/*
*@author:  Ximena Patricia García Magdaleno
* BenefitsState.hpp
* State Machine Tier Three.3- Benefits State
*
* Follow the line with qtrRear and unload at each box until both are done.
* With Benefits::kBluePriority, blue goes first: a red box found first is
* remembered (redPending_) and visited after blue.
* A side line (FL going left, FR going right) is the end of the line: turn around.
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class BenefitsState {
public:
    void begin() {
        stage_ = Stage::SEARCH;
        stageStartMs_ = 0;
        rearCorrFiltered = 0.0f;

        dir_ = Constants::StateMachine::Benefits::kStartDir;
        blueDone_ = false;
        redDone_ = false;
        redPending_ = false;
        redSeenDir_ = 0;
        turns_ = 0;

        benefitRequestMs = 0;
        benefitColour = VisionLink::kNoBenefit;
        approachColour = VisionLink::kNoBenefit;
        benefitArmed = true;
        benefitOpened = false;
        benefitOpenedMs = 0;
        benefitLastSeenMs = 0;
        benefitApproachMs = 0;
    }

    // FL/FR: side lines that end the line when going left (+vy) / right (-vy).
    void update(uint32_t now, bool FL, bool FR, bool& transitionToStop) {
        namespace cfg = Constants::StateMachine::Benefits;
        transitionToStop = false;

        // Doors open only through openBenefit(); the stream just reports the box.
        vision.setBenefitsOnRequest(true);

        if (vision.hasError()) {
            LARC.stop();
            vision.closeBenefit();
            vision.resetGuards();
            vision.stop();
            transitionToStop = true;
            return;
        }

        // Wait in place until the Orin reports fresh BENEFITS results,
        // re-sending START_BENEFITS every second.
        if (!vision.inBenefitsPhase() || !vision.isBenefitsRunning()) {
            LARC.stop();
            if (benefitRequestMs == 0 || (now - benefitRequestMs) >= cfg::kVisionRetryMs) {
                vision.resetGuards();
                vision.startBenefits();
                benefitRequestMs = now;
            }
            return;
        }

        switch (stage_) {
            case Stage::SEARCH:
                search(now, FL, FR, transitionToStop);
                break;

            case Stage::TURN:
                LARC.stop();
                if ((now - stageStartMs_) >= cfg::kTurnStopMs) {
                    dir_ = -dir_;
                    stage_ = Stage::SEARCH;
                }
                break;

            case Stage::ADVANCE:
                advance(now, FL, FR);
                break;

            case Stage::AT_BOX:
                atBox(now, transitionToStop);
                break;
        }
    }

    bool blueDone()   const { return blueDone_; }
    bool redPending() const { return redPending_; }

private:
    // ADVANCE: box confirmed, keep driving kBoxAdvanceMs so the door ends up centred on it.
    enum class Stage : uint8_t { SEARCH, TURN, ADVANCE, AT_BOX };

    // VisionLink::seenBenefit(): 0 = BLUE door (blue + black), 1 = RED door
    static constexpr uint8_t kBlue = 0;
    static constexpr uint8_t kRed  = 1;

    Stage    stage_ = Stage::SEARCH;
    uint32_t stageStartMs_ = 0;
    float    rearCorrFiltered = 0.0f;

    int8_t   dir_ = 1;         // +1 = left (+vy), -1 = right (-vy)
    bool     blueDone_ = false;   // blue door opened: blue boxes are driven past from now on
    bool     redDone_ = false;
    bool     redPending_ = false; // red box seen before blue was unloaded (kBluePriority only)
    int8_t   redSeenDir_ = 0;     // travel direction when the red box was last passed
    uint8_t  turns_ = 0;

    uint32_t benefitRequestMs = 0;
    uint8_t  benefitColour = VisionLink::kNoBenefit;
    bool     benefitArmed = true;
    bool     benefitOpened = false;
    uint32_t benefitOpenedMs = 0;
    uint32_t benefitLastSeenMs = 0; // last time any part of a box was in view while driving
    uint32_t benefitApproachMs = 0; // when the box became centred while driving, 0 = not centred
    uint8_t  approachColour = VisionLink::kNoBenefit;

    // A box worth stopping for: the priority target, or (no priority) any colour not unloaded yet.
    bool wanted(uint8_t colour) const {
        if (Constants::StateMachine::Benefits::kBluePriority)
            return colour == (blueDone_ ? kRed : kBlue);
        return colour == kBlue ? !blueDone_ : !redDone_;
    }

    void search(uint32_t now, bool FL, bool FR, bool& transitionToStop) {
        using namespace StateCommon;
        using namespace Constants::ServoConfig;
        namespace cfg = Constants::StateMachine::Benefits;

        if (dir_ > 0 ? FL : FR) {
            LARC.stop();
            // Safety: the box never showed up, do not bounce along the line forever.
            if (++turns_ > cfg::kMaxTurns) {
                transitionToStop = true;
                return;
            }
            stage_ = Stage::TURN;
            stageStartMs_ = now;
            return;
        }

        drive();

        // Re-arm only once the box has FULLY left the camera for kBenefitRearmMs,
        // so a box sliding out slowly cannot cause a second stop.
        if (vision.benefitVisible())
            benefitLastSeenMs = now;
        else if (now - benefitLastSeenMs >= kBenefitRearmMs)
            benefitArmed = true;

        // The same colour must stay centred for kBenefitApproachMs before we act on it
        // (confirmed while driving: after ADVANCE the box is no longer centred).
        if (!vision.benefitSeen()) {
            benefitApproachMs = 0;
        } else if (benefitApproachMs == 0 || vision.seenBenefit() != approachColour) {
            benefitApproachMs = now;
            approachColour = vision.seenBenefit();
        }

        if (!benefitArmed || benefitApproachMs == 0 || (now - benefitApproachMs) < kBenefitApproachMs)
            return;

        const uint8_t colour = approachColour;
        benefitArmed = false;
        benefitApproachMs = 0;

        if (!wanted(colour)) {
            // Red before blue (kBluePriority): remember it and keep looking for blue.
            if (cfg::kBluePriority && colour == kRed) {
                redPending_ = true;
                redSeenDir_ = dir_;
            }
            return;
        }

        benefitOpened = false;
        benefitColour = colour;
        stageStartMs_ = now;
        stage_ = Stage::ADVANCE;
    }

    // Line following along the boxes with qtrRear, in the current direction.
    void drive() {
        using namespace StateCommon;
        const float corrTarget = rearCornerCorrTarget();
        rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;
        LARC.setTranslation(rearCorrFiltered, dir_ * kVelocity);
    }

    // Box confirmed: keep going kBoxAdvanceMs so the door lines up, then stop and unload.
    // A side line cuts it short (do not drive off the end of the line).
    void advance(uint32_t now, bool FL, bool FR) {
        namespace cfg = Constants::StateMachine::Benefits;

        if ((now - stageStartMs_) < cfg::kBoxAdvanceMs && !(dir_ > 0 ? FL : FR)) {
            drive();
            return;
        }

        LARC.stop();
        stageStartMs_ = now;
        stage_ = Stage::AT_BOX;
    }

    // Stopped past the box: open the door of the colour confirmed while driving.
    void atBox(uint32_t now, bool& transitionToStop) {
        using namespace Constants::ServoConfig;

        LARC.stop();

        if (!benefitOpened) {
            vision.openBenefit(benefitColour);
            benefitOpened = vision.benefitHeld();
            benefitOpenedMs = now;
        }

        if (benefitOpened && vision.benefitHeld() && (now - benefitOpenedMs) >= kBenefitOpenMs)
            vision.closeBenefit();

        const bool stopDone = benefitOpened ? (now - benefitOpenedMs) >= kBenefitStopMs
                                            : (now - stageStartMs_) >= kBenefitStopMs;
        if (!stopDone)
            return;

        vision.closeBenefit();

        // The door could not open (vision left the BENEFITS phase): stay armed for the next box.
        if (!benefitOpened) {
            resumeSearch(now);
            benefitArmed = true;
            return;
        }

        if (benefitColour == kBlue)
            blueDone_ = true;
        else
            redDone_ = true;

        if (blueDone_ && redDone_) {
            transitionToStop = true;
            return;
        }

        // Red was passed going this way, so it is behind us.
        if (redPending_ && redSeenDir_ == dir_)
            dir_ = -dir_;
        resumeSearch(now);
    }

    void resumeSearch(uint32_t now) {
        benefitLastSeenMs = now; // the box must vanish for kBenefitRearmMs from here
        stage_ = Stage::SEARCH;
    }
};
