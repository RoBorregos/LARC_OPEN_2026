#include "Perception.hpp"
#include "constants.h"

void Perception::update(uint32_t now, bool readTof)
{
    ir.update();
    if (readTof)
    {
        tofLeft.update();
        tofRight.update();
        tofBackLeft.update();
        tofBackRight.update();
    }
    qtrFront.update();
    qtrRear.update();

    updateIR();
    updateQTR();
    updateToF(now, readTof);
}

void Perception::updateIR()
{
    snap_.FL = ir.getState(IRLine::FL);
    snap_.FR = ir.getState(IRLine::FR);
    snap_.BL = ir.getState(IRLine::BL);
    snap_.BR = ir.getState(IRLine::BR);

    snap_.frontLeftDetectedLine  = snap_.FL;
    snap_.frontRightDetectedLine = snap_.FR;
    snap_.backLeftDetectedLine   = snap_.BL;
    snap_.backRightDetectedLine  = snap_.BR;
    snap_.frontDetectedLine = snap_.FL || snap_.FR;
    snap_.backDetected      = snap_.BL || snap_.BR;
    snap_.leftDetectedPool  = snap_.FL || snap_.BL;
    snap_.rightDetectedPool = snap_.FR || snap_.BR;
}

void Perception::updateQTR()
{
    snap_.linePos = qtrFront.getPosition();
    snap_.onLine  = qtrFront.onLine(Constants::StateMachine::Common::kFrontOnLineThreshold);
}

void Perception::updateToF(uint32_t now, bool readTof)
{
    if (!readTof)
    {
        tofPaused_         = true;
        frontLatch_        = ObstacleLatch{};
        rearLatch_         = ObstacleLatch{};
        snap_.obstacle     = false;
        snap_.rearObstacle = false;
        return;
    }

    // The first reading after a pause is the one left waiting in the sensor
    // since the pause began: ignore obstacles until fresh data has come in.
    if (tofPaused_)
    {
        tofPaused_   = false;
        tofResumeMs_ = now;
    }
    const bool resuming = tofResumeMs_ != 0 && (now - tofResumeMs_) < kTofResumeBlankMs;

    if (tofReadyTimestamp_ == 0 &&
        (tofLeft.isValid() || tofRight.isValid() || tofBackLeft.isValid() || tofBackRight.isValid()))
        tofReadyTimestamp_ = now;

    const bool tofReady = tofReadyTimestamp_ != 0 &&
                          (now - tofReadyTimestamp_) > kTofWarmupMs;
    snap_.tofReady = tofReady;

    auto seesObstacle = [&](const ToF &tof)
    {
        return !Constants::PerceptionConfig::kIgnoreObstacles &&
               tofReady && !resuming && tof.isValid() && tof.getDistanceMm() < kObstacleDistanceMm;
    };

    snap_.obstacle     = frontLatch_.update(now, seesObstacle(tofLeft) || seesObstacle(tofRight));
    snap_.rearObstacle = rearLatch_.update(now, seesObstacle(tofBackLeft) || seesObstacle(tofBackRight));
}

bool Perception::ObstacleLatch::update(uint32_t now, bool seenNow)
{
    if (!latched)
    {
        if (!seenNow)
        {
            detectStartMs = 0;
            return false;
        }
        if (detectStartMs == 0)
            detectStartMs = now;
        if ((now - detectStartMs) >= kConfirmMs)
        {
            latched       = true;
            clearStartMs  = 0;
            detectStartMs = 0;
        }
        return latched;
    }

    if (seenNow)
    {
        clearStartMs = 0;
    }
    else
    {
        if (clearStartMs == 0)
            clearStartMs = now;
        if ((now - clearStartMs) >= kReleaseMs)
        {
            latched      = false;
            clearStartMs = 0;
        }
    }
    return latched;
}
