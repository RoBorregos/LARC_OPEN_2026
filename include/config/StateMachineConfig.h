/**
 * @file StateMachineConfig.h
 * @author Ximena Patricia García Magdaleno
 *
 * @brief LARCStateMachine tuning: speeds and timings, one namespace per state.
 *        BENEFITS door/stop timings live in ServoConfig.
 */

#pragma once

#include <Arduino.h>
#include "config/DriveConfig.h"

namespace Constants
{
    namespace StateMachine
    {
        // Shared by several states
        namespace Common
        {
            static constexpr float kVelocity      = 0.25f;
            static constexpr float kBaseSpeed     = Constants::PID::kcurrentVelocity;
            static constexpr float kManeuverSpeed = 0.30f; // backups, strafes and line corrections

            static constexpr float kCornerSetpoint  = 2900.0f;
            static constexpr float kCornerKp        = 0.00012f;
            static constexpr float kCornerCorrMax   = 0.20f;
            static constexpr float kCornerCorrAlpha = 0.15f;
            static constexpr float kRearCorrAlpha   = 0.15f;

            static constexpr uint32_t kStartIgnoreTimeMs    = 4500; // Time to ignore IR's at the START point
            static constexpr uint32_t kClearDelayMs         = 500;  // Obstacle clear this long -> back to FORWARD
            static constexpr uint32_t kNoObstacleToCornerMs = 1000; // No obstacle this long -> leave the pools state

            static constexpr uint32_t kMinAvoidTimeMs       = 250;
            static constexpr uint32_t kSideDetectHoldMs     = 80;
            static constexpr uint16_t kTooCloseAvoidLeftMm  = 120;
            static constexpr uint16_t kTooCloseAvoidRightMm = 100;
        } // namespace Common

        namespace Elevator
        {
            static constexpr bool kLimitSwitchConnected = true;
            // External pull-ups: pressed reads HIGH (same as test_elevator). Flip to LOW if inverted.
            static constexpr int  kLimitPressedLevel = HIGH;

            // Elevator::ElevatorPosition() states, same as test_elevator
            static constexpr int kElevatorStop = 0;
            static constexpr int kElevatorUp   = 1;
            static constexpr int kElevatorDown = 2;
        } // namespace Elevator

        // IrSideLatch: first IR of a side latches, the other one triggers the correction
        namespace IrLatch
        {
            static constexpr uint32_t kLatchTimeoutMs = 800;
            static constexpr uint32_t kLatchIrOffMs   = 150;
        } // namespace IrLatch

        // QtrEntryTracker settings shared by LOOKFORLINE and LOOKFORLINEBACKWARDS
        namespace QtrTracker
        {
            static constexpr uint16_t kOffThresholdMargin = 80; // offThreshold = kBinaryThreshold - this
            static constexpr float    kEntryMaxProgress   = 1.0f;
            static constexpr float    kCenterTolerance    = 0.75f;
            static constexpr float    kMaxRegress         = 0.6f;
            static constexpr uint8_t  kConfirmReads       = 2;
            static constexpr uint8_t  kOnReads            = 3;
            static constexpr uint8_t  kLostReads          = 5;
        } // namespace QtrTracker

        namespace Start
        {
            static constexpr uint32_t kElevatorUpMs   = 9000;
            static constexpr uint32_t kElevatorStopMs = 700;
            static constexpr float    kForwardSpeed   = 0.28f;
        } // namespace Start

        namespace Pool
        {
            static constexpr uint32_t kLineCorrectionMs   = 120;
            static constexpr uint32_t kJustEnteredLeftMs  = 150; // ignore the border right after entering AVOID_LEFT
            static constexpr uint32_t kJustEnteredRightMs = 100; // ignore the border right after entering AVOID_RIGHT
        } // namespace Pool

        namespace LookForLine
        {
            static constexpr uint32_t kBackupMs        = 200; // stage 0 and stage 1, each
            static constexpr uint32_t kForwardMs       = 300; // stage 2
            static constexpr uint32_t kBorderCorrectMs = 150;
            static constexpr uint32_t kIrConfirmMs     = 400;

            // qtrFront: C0..C6 (indices 0..6), C6 is the frontmost sensor.
            // Set to false if the line enters through C0 when going forward.
            static constexpr bool  kEntryAtHighIndex = true;
            static constexpr float kCenterProgress   = 3.0f;
        } // namespace LookForLine

        namespace LookForCorner
        {
            static constexpr uint32_t kBeansRetryMs = 1000; // re-send START_BEANS while waiting
            static constexpr uint32_t kSoftStartMs  = 500;
        } // namespace LookForCorner

        namespace Beans
        {
            static constexpr uint32_t kLostLineTimeoutMs = 1200;
            static constexpr uint32_t kCornerStopMs      = 1000;
        } // namespace Beans

        namespace PoolsGoBack
        {
            static constexpr uint32_t kInitBackMs = 500;
            static constexpr uint32_t kInitLeftMs = 500;
        } // namespace PoolsGoBack

        namespace LookForLineBackwards
        {
            static constexpr uint32_t kBorderCorrectMs = 150;

            // qtrRear: C8..C13 (indices 0..5), the line enters through C8 when going backwards.
            static constexpr bool  kEntryAtHighIndex = false;
            static constexpr float kCenterProgress   = 2.5f;
        } // namespace LookForLineBackwards

        namespace BenefitsStartCorner
        {
            static constexpr uint32_t kCornerStopMs = 1000;
        } // namespace BenefitsStartCorner

        namespace Benefits
        {
            static constexpr uint32_t kVisionRetryMs = 1000; // re-send START_BENEFITS while waiting
            static constexpr uint32_t kCornerStopMs  = 1000;
        } // namespace Benefits

        namespace Debug
        {
            static constexpr uint32_t kPrintPeriodMs = 100;
        } // namespace Debug
    } // namespace StateMachine
} // namespace Constants
