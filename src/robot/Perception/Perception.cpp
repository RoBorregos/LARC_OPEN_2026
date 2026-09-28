#include "Perception.hpp"
#include "constants.h"

void Perception::update(uint32_t now)
{
    updateIR();
    updateQTR();
    updateToF(now);
    updateEncoders(now);
}

void Perception::updateIR()
{
    snap_.FL = ir.getState(IRLine::FL);
    snap_.FR = ir.getState(IRLine::FR);
    snap_.BL = ir.getState(IRLine::BL);
    snap_.BR = ir.getState(IRLine::BR);

    snap_.frontDetectedLine = snap_.FL || snap_.FR;
    snap_.backDetected      = snap_.BL || snap_.BR;
    snap_.leftDetectedPool  = snap_.FL || snap_.BL;
    snap_.rightDetectedPool = snap_.FR || snap_.BR;
}

void Perception::updateQTR()
{
    snap_.linePos = qtrFront.getPosition();
    snap_.onLine  = qtrFront.onLine();

    const float lineCorr = linePID.update(snap_.linePos, Constants::LineFollower::kSetpoint);
    snap_.vx = -lineCorr;
}

void Perception::updateToF(uint32_t now)
{
    if (tofReadyTimestamp_ == 0 && (tofLeft.isValid() || tofRight.isValid()))
        tofReadyTimestamp_ = now;

    const bool tofReady = tofReadyTimestamp_ != 0 &&
                          (now - tofReadyTimestamp_) > kTofWarmupMs;

    snap_.obstacleLeft  = tofReady && tofLeft.isValid()  && tofLeft.getDistanceCm()  < kObstacleDistanceCm;
    snap_.obstacleRight = tofReady && tofRight.isValid() && tofRight.getDistanceCm() < kObstacleDistanceCm;

    if (!obstacleLatched_)
    {
        if (snap_.obstacleLeft || snap_.obstacleRight)
        {
            if (obstacleDetectStartMs_ == 0)
                obstacleDetectStartMs_ = now;

            if ((now - obstacleDetectStartMs_) >= kObstacleConfirmMs)
            {
                obstacleLatched_       = true;
                obstacleClearStartMs_  = 0;
                obstacleDetectStartMs_ = 0;
            }
        }
        else
        {
            obstacleDetectStartMs_ = 0; // reset si deja de verse
        }
    }
    else
    {
        if (snap_.obstacleLeft || snap_.obstacleRight)
        {
            obstacleClearStartMs_ = 0;
        }
        else
        {
            if (obstacleClearStartMs_ == 0)
                obstacleClearStartMs_ = now;

            if ((now - obstacleClearStartMs_) >= kObstacleReleaseMs)
            {
                obstacleLatched_      = false;
                obstacleClearStartMs_ = 0;
            }
        }
    }

    snap_.obstacle = obstacleLatched_;
}

void Perception::updateEncoders(uint32_t /*now*/)
{
    // TODO: once ENCODER exposes per-wheel deltas, compute wheelStalled and
    // distanceTraveledCm here so Pool/PoolsGoBack can gate on wheel slip
    // instead of relying only on ToF + timers.
}
