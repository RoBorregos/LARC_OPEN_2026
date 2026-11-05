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
        lineCorr_.reset();
        wallPd_.reset();
        wallLastMs_ = 0;
        wallSeenMs_ = 0;
        wallSeenMm_ = 0;
        wallSettled_ = false;

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
    // rearLeftMm/rearRightMm: LL/LR distance (0 = no reading), drives vx while searching.
    void update(uint32_t now, bool FL, bool FR, uint16_t rearLeftMm, uint16_t rearRightMm, bool& transitionToStop) {
        namespace cfg = Constants::StateMachine::Benefits;
        transitionToStop = false;

        // Closer of the sensors that see a box (0 or > kWallNoBoxMm = no box)
        rearDistValid_ = false;
        rearDistMm_    = 0;
        for (const uint16_t d : {rearLeftMm, rearRightMm}) {
            if (d != 0 && d <= cfg::kWallNoBoxMm && (!rearDistValid_ || d < rearDistMm_)) {
                rearDistMm_    = d;
                rearDistValid_ = true;
            }
        }
        if (rearDistValid_) {
            wallSeenMm_ = rearDistMm_;
            wallSeenMs_ = now;
        }

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
        // re-sending START_BENEFITS every second. vx keeps correcting meanwhile.
        if (!vision.inBenefitsPhase() || !vision.isBenefitsRunning()) {
            drive(now, 0.0f);
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

    // The doors share the I2C bus with the ToFs: no ToF reads while unloading.
    bool wantsTof() const { return Constants::StateMachine::Benefits::kUseRearTof && stage_ != Stage::AT_BOX; }

    // Debug: what drive() used last. QTR = no box within kWallNoBoxMm.
    enum class WallMode : uint8_t { QTR, TOF, TOO_CLOSE };
    WallMode wallMode()   const { return wallMode_; }
    uint16_t wallDistMm() const { return rearDistValid_ ? rearDistMm_ : 0; }
    float    lastVx()     const { return lastVx_; }

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
    StateCommon::LineCorrection lineCorr_{qtrRear, Constants::StateMachine::Common::kQtrRearSetpoint};

    static constexpr uint32_t kWallMaxGapMs = 100; // longer without a ToF update -> restart the wall PD
    static constexpr LinePD::Gains kWallGains = {
        Constants::StateMachine::Benefits::kWallKp, 0.0f, 0.0f, 0.03f,
        Constants::StateMachine::Benefits::kWallCorrMax, Constants::StateMachine::Benefits::kWallSlewPerS};
    LinePD   wallPd_{kWallGains};
    uint32_t wallLastMs_    = 0;
    bool     rearDistValid_ = false;
    uint16_t rearDistMm_    = 0;
    WallMode wallMode_      = WallMode::QTR;
    uint32_t wallSeenMs_    = 0; // last valid box reading
    uint16_t wallSeenMm_    = 0;
    bool     wallSettled_   = false; // within the target band, vx held at 0 (hysteresis)
    float    lastVx_        = 0.0f;

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

        drive(now, dir_ * kVelocity);

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

    // Drive along the boxes in the current direction. vx keeps the rear ToF distance at
    // kWallTargetMm; without a valid ToF reading it falls back to the qtrRear line.
    void drive(uint32_t now, float vy) {
        using namespace StateCommon;
        namespace cfg = Constants::StateMachine::Benefits;

        // Box reading lost right after being close: the sensor is too close to measure.
        const bool lostClose = !rearDistValid_ && wallSeenMs_ != 0 &&
                               wallSeenMm_ < cfg::kWallTargetMm &&
                               (now - wallSeenMs_) < cfg::kWallLostCloseMs;
        // Box seen moments ago (ToF blank after unloading, a dropped reading): hold vx at 0
        // instead of the qtrRear setpoint, which sits elsewhere and would pull the robot forward.
        const bool boxRecent = !rearDistValid_ && !lostClose && wallSeenMs_ != 0 &&
                               (now - wallSeenMs_) < cfg::kWallHoldMs;
        const bool useTof = cfg::kUseRearTof && (rearDistValid_ || lostClose || boxRecent);

        float vx;
        if (useTof && wallLastMs_ != 0 && now == wallLastMs_) {
            vx = lastVx_;
        } else if (useTof) {
            wallMode_ = WallMode::TOF;
            const bool restart = wallLastMs_ == 0 || (now - wallLastMs_) > kWallMaxGapMs;
            if (restart)
                wallPd_.reset();
            const float dt = restart ? 0.01f : (now - wallLastMs_) / 1000.0f;
            wallLastMs_ = now;

            if (boxRecent) {
                vx = -wallPd_.hold(0.0f, dt);
            } else if (lostClose || rearDistMm_ < cfg::kWallMinMm) {
                // Too close: move away (+vx) at full correction
                wallMode_ = WallMode::TOO_CLOSE;
                wallSettled_ = false;
                vx = -wallPd_.hold(-cfg::kWallCloseVx, dt);
            } else {
                // > 0: too far from the boxes -> back up (-vx)
                const float errMm  = (float)rearDistMm_ - (float)cfg::kWallTargetMm;
                if (wallSettled_ ? fabsf(errMm) > cfg::kWallReleaseMm
                                 : fabsf(errMm) <= cfg::kWallDeadbandMm)
                    wallSettled_ = !wallSettled_;
                const float shrunk = (errMm > 0.0f) ? max(errMm - cfg::kWallDeadbandMm, 0.0f)
                                                    : min(errMm + cfg::kWallDeadbandMm, 0.0f);
                if (wallSettled_) {
                    vx = -wallPd_.hold(0.0f, dt);
                } else {
                    float u = wallPd_.update(shrunk, dt);
                    if (shrunk != 0.0f && fabsf(u) < cfg::kWallMinVx)
                        u = (shrunk > 0.0f) ? cfg::kWallMinVx : -cfg::kWallMinVx;
                    vx = -u;
                }
            }
        } else {
            wallMode_ = WallMode::QTR;
            vx = -lineCorr_.update(now, rearOnLine());
        }
        lastVx_ = vx;
        LARC.setTranslation(vx, vy);
    }

    // Box confirmed: keep going kBoxAdvanceMs so the door lines up, then stop and unload.
    // A side line cuts it short (do not drive off the end of the line).
    void advance(uint32_t now, bool FL, bool FR) {
        namespace cfg = Constants::StateMachine::Benefits;

        if ((now - stageStartMs_) < cfg::kBoxAdvanceMs && !(dir_ > 0 ? FL : FR)) {
            drive(now, dir_ * StateCommon::kVelocity);
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
        if (wallSeenMs_ != 0)
            wallSeenMs_ = now;   // no ToF reads in AT_BOX: hold vx through the resume blank
        stage_ = Stage::SEARCH;
    }
};
