#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

// Groups the per-tick sensor fusion that used to live inline in
// LARCStateMachine::update(): ToF (VLX) obstacle latching, IR line grouping,
// and front QTR line-follow error. Perception::update() is called once per
// tick and the resulting snapshot is shared by every state that needs it
// (POOL, POOLSGOBACK, LOOKFORLINE, ...).
struct PerceptionSnapshot
{
    // IR (raw per-corner + grouped)
    bool FL = false, FR = false, BL = false, BR = false;
    bool frontDetectedLine = false; // FL || FR
    bool backDetected      = false; // BL || BR
    bool leftDetectedPool  = false; // FL || BL
    bool rightDetectedPool = false; // FR || BR

    // Front QTR line-follow
    int   linePos = 0;
    bool  onLine  = false;
    float vx      = 0.0f; // -linePID output, forward velocity correction

    // ToF (VLX) obstacle, debounced/latched
    bool obstacle      = false; // latched
    bool obstacleLeft  = false; // instantaneous
    bool obstacleRight = false; // instantaneous

    // Encoders: ENCODER is a stub for now. These fields are reserved so
    // states can start consuming wheel-based signals (stall/slip detection,
    // distance gating) without another round of plumbing once it's wired up.
    bool  wheelStalled       = false;
    float distanceTraveledCm = 0.0f;
};

class Perception
{
public:
    void update(uint32_t now);

    const PerceptionSnapshot &get() const { return snap_; }

private:
    PerceptionSnapshot snap_;

    // ToF obstacle latch state (see updateToF)
    uint32_t tofReadyTimestamp_     = 0;
    bool     obstacleLatched_       = false;
    uint32_t obstacleClearStartMs_  = 0;
    uint32_t obstacleDetectStartMs_ = 0;

    static constexpr uint32_t kTofWarmupMs        = 500;
    static constexpr uint32_t kObstacleReleaseMs  = 400;
    static constexpr uint32_t kObstacleConfirmMs  = 0;
    static constexpr float    kObstacleDistanceCm = 20.0f;

    void updateIR();
    void updateQTR();
    void updateToF(uint32_t now);
    void updateEncoders(uint32_t now);
};
