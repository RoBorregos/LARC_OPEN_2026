#pragma once
#include <Arduino.h>
#include "constants.h"
#include "robot/instances/instances.hpp"

struct PerceptionSnapshot
{
    // IR
    bool FL = false, FR = false, BL = false, BR = false;
    bool frontLeftDetectedLine  = false; // FL
    bool frontRightDetectedLine = false; // FR
    bool backLeftDetectedLine   = false; // BL
    bool backRightDetectedLine  = false; // BR
    bool frontDetectedLine = false;      // FL || FR
    bool backDetected      = false;      // BL || BR
    bool leftDetectedPool  = false;      // FL || BL
    bool rightDetectedPool = false;      // FR || BR

    // Front QTR
    int   linePos = 0;
    bool  onLine  = false;

    // ToF (mm)
    bool tofReady     = false;
    bool obstacle     = false; // UL/UR -> POOL
    bool rearObstacle = false; // LL/LR -> POOLSGOBACK
};

class Perception
{
public:
    static constexpr uint16_t kTofMaxRangeMm      = Constants::PerceptionConfig::kTofMaxRangeMm;
    static constexpr uint16_t kObstacleDistanceMm = Constants::PerceptionConfig::kObstacleDistanceMm;

    // readTof false skips the ToF I2C reads (keeps the shared bus free for the servos);
    // obstacle/rearObstacle then read false.
    void update(uint32_t now, bool readTof = true);

    const PerceptionSnapshot &get() const { return snap_; }

private:
    struct ObstacleLatch
    {
        static constexpr uint32_t kReleaseMs = Constants::PerceptionConfig::kObstacleReleaseMs;
        static constexpr uint32_t kConfirmMs = Constants::PerceptionConfig::kObstacleConfirmMs;

        bool     latched       = false;
        uint32_t clearStartMs  = 0;
        uint32_t detectStartMs = 0;

        bool update(uint32_t now, bool seenNow);
    };

    static constexpr uint32_t kTofWarmupMs     = Constants::PerceptionConfig::kTofWarmupMs;
    static constexpr uint32_t kTofResumeBlankMs = Constants::PerceptionConfig::kTofResumeBlankMs;

    PerceptionSnapshot snap_;
    uint32_t tofReadyTimestamp_ = 0;
    bool     tofPaused_         = false;
    uint32_t tofResumeMs_       = 0;
    ObstacleLatch frontLatch_;
    ObstacleLatch rearLatch_;

    void updateIR();
    void updateQTR();
    void updateToF(uint32_t now, bool readTof);
};
