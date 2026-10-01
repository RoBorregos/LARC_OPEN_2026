#include "Perception.hpp"
#include "constants.h"

void Perception::update(uint32_t now)
{
    ir.update();
    tofLeft.update();
    tofRight.update();
    tofBackLeft.update();
    tofBackRight.update();
    qtrFront.update();
    qtrRear.update();

    updateIR();
    updateQTR();
    updateToF(now);
}

void Perception::updateIR()
{
    snap_.FL = ir.getState(IRLine::FL);
    snap_.FR = ir.getState(IRLine::FR);
    snap_.BL = ir.getState(IRLine::BL);
    snap_.BR = ir.getState(IRLine::BR);

    snap_.frontLeftDetectedLine  = snap_.FL;
    snap_.frontRightDetectedLine = snap_.FR;
    snap_.backLeftDetectedLine   = snap_.BR;
    snap_.backRightDetectedLine  = snap_.BL;
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
    if (tofReadyTimestamp_ == 0 &&
        (tofLeft.isValid() || tofRight.isValid() || tofBackLeft.isValid() || tofBackRight.isValid()))
        tofReadyTimestamp_ = now;

    const bool tofReady = tofReadyTimestamp_ != 0 &&
                          (now - tofReadyTimestamp_) > kTofWarmupMs;
    snap_.tofReady = tofReady;

    auto seesObstacle = [&](const ToF &tof)
    {
        return tofReady && tof.isValid() && tof.getDistanceMm() < kObstacleDistanceMm;
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
