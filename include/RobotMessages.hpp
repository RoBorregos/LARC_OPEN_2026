#pragma once

#include <Arduino.h>

/**
 * Data exchanged between the future SensorTask and RobotTask.
 *
 * For now this documents the boundary. The next migration step is to make
 * the state machine consume this snapshot instead of reading drivers.
 */
struct SensorSnapshot
{
    uint32_t timestampMs = 0;

    int16_t frontLinePosition = 0;
    int16_t rearLinePosition = 0;
    bool frontOnLine = false;
    bool rearOnLine = false;

    bool lineFrontLeft = false;
    bool lineFrontRight = false;
    bool lineBackLeft = false;
    bool lineBackRight = false;

    uint16_t tofLeftMm = 0xFFFF;
    uint16_t tofRightMm = 0xFFFF;
    float yawRad = 0.0f;

    bool healthy = false;
};

/**
 * Command exchanged between the future RobotTask and DriveControlTask.
 * Values are expressed in wheel RPM coordinates used by OdomMovement.
 */
struct MotionCommand
{
    uint32_t timestampMs = 0;
    float upperLeftRpm = 0.0f;
    float upperRightRpm = 0.0f;
    float lowerLeftRpm = 0.0f;
    float lowerRightRpm = 0.0f;
    bool enabled = false;
};
