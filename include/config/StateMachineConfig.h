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

            // Line center for each QTR, in that QTR's own lPos units (debug print).
            static constexpr float kQtrFrontSetpoint = 2587.0f; // qtrFront: 0..6000
            static constexpr float kQtrRearSetpoint  = 2583.3f; // qtrRear:  0..5000 (same as the old 2900 mirrored)
            static constexpr float kCornerKp        = 0.00012f; // qtrRear
            static constexpr float kFrontCornerKp   = 0.00008f; // qtrFront; lower than rear because it wobbled at 0.00012
            static constexpr float kFrontCornerDeadband = 150.0f; // lPos error ignored around the setpoint
            static constexpr float kCornerCorrMax   = 0.20f;
            static constexpr float kCornerCorrAlpha = 0.15f;
            static constexpr float kRearCorrAlpha   = 0.15f;

            static constexpr uint32_t kStartIgnoreTimeMs    = 4500; // Time to ignore IR's at the START point
            static constexpr uint32_t kClearDelayMs         = 500;  // Obstacle clear this long -> back to FORWARD
            static constexpr uint32_t kNoObstacleToCornerMs = 1000; // No obstacle this long -> leave the pools state
            static constexpr uint32_t kKeepRightResetMs     = 800;  // FORWARD clear this long -> next obstacle is avoided left again

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
            static constexpr uint32_t kElevatorStopMs = 700;  // minimum stop before driving; also waits for a stable yaw
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
            // Until the first pool is seen, keep backing up with the rear ToFs this long
            // before giving up (the pools are far from the BEANS corner).
            static constexpr uint32_t kFirstPoolTimeoutMs = 4500;
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

        // Robot stays still in begin() until the BNO085 yaw stops moving, then latches the yaw target.
        namespace ImuSettle
        {
            static constexpr uint32_t kMinMs          = 1500; // always wait at least this long
            static constexpr uint32_t kMaxMs          = 4000; // give up waiting for stability after this
            static constexpr uint32_t kStableWindowMs = 500;  // yaw must stay within kStableDeg this long
            static constexpr float    kStableDeg      = 0.3f;
        } // namespace ImuSettle

        namespace Debug
        {
            static constexpr uint32_t kPrintPeriodMs = 100;
        } // namespace Debug
    } // namespace StateMachine
} // namespace Constants
