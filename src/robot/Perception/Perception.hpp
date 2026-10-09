#pragma once
#include <Arduino.h>
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
    float vx      = 0.0f;

    // ToF (mm)
    bool tofReady     = false;
    bool obstacle     = false; // UL/UR -> POOL
    bool rearObstacle = false; // LL/LR -> POOLSGOBACK
};

class Perception
{
public:
    static constexpr uint16_t kTofMaxRangeMm      = 1000;
    static constexpr uint16_t kObstacleDistanceMm = 215;

    void update(uint32_t now);

    const PerceptionSnapshot &get() const { return snap_; }

private:
    struct ObstacleLatch
    {
        static constexpr uint32_t kReleaseMs = 400;
        static constexpr uint32_t kConfirmMs = 0;

        bool     latched       = false;
        uint32_t clearStartMs  = 0;
        uint32_t detectStartMs = 0;

        bool update(uint32_t now, bool seenNow);
    };

    static constexpr uint32_t kTofWarmupMs = 500;

    PerceptionSnapshot snap_;
    uint32_t tofReadyTimestamp_ = 0;
    ObstacleLatch frontLatch_;
    ObstacleLatch rearLatch_;

    void updateIR();
    void updateQTR();
    void updateToF(uint32_t now);
};