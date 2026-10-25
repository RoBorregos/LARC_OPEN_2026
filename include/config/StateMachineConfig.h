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
            /*
                1) Como calibrarlo: En activa true en Teleplot o en debug prints true:
                2) Despues (ya calibrado el qtr) colora el robot de acuerdo a que tan cerca lo quieres del arbol
                3) Sumale + 1500 a ese setPoint y ese valor colocalo en kQtrFrontSetpoint

                 Por que: Ya que la correccion en LookForCorner es mas alejada
                 y en Beans se suele pegar mas el qtr al arbol, si dejas el valor
                 real al que tanto quieres el robot cerca del arbol, la realidad es que \
                 en el estad BEANS, va a chocar. Por eso sumas los 1500 y eso es lo que pones
                 asi en LookForCorner esta mas despegado del arbol, y en BEANS va a la distancia que lo querias.
                 - Conveniendte porque asi no choca con los arboles cuando recien llega a linea
                    de LookForLine.

                    Ejemplo: Originalmente lo queria a 1500 el SetPoint, pero terino usando 
                            3000 como el actual SetPoint value.
            */
            static constexpr float kQtrFrontSetpoint = 3000.0f; //  1500 cerca // Entra por C6 y termina en C0; es decir si quiers que este mascerca del arbol que SetPoint vaya en rango de 1000 (mas cerca)-2000 (mas lejos)
            static constexpr float kQtrRearSetpoint  = 2583.3f; // qtrRear:  0..5000 (same as the old 2900 mirrored)
            static constexpr float kCornerKp        = 0.00012f; // qtrRear
            static constexpr float kCornerCorrMax   = 0.20f;
            static constexpr float kRearCorrAlpha   = 0.15f;

            // qtrFront line correction (StateCommon::FrontLineCorrection), error in mm
            static constexpr float    kQtrPitchMm           = 9.525f; // QTR-8A sensor spacing
            static constexpr uint16_t kFrontFloorNorm       = 300;    // background norm ignored by the position (C4 floor ~210)
            static constexpr uint16_t kFrontOnLineThreshold = 400;    // floor reads up to ~210 on C4
            static constexpr float    kFrontLineKp          = 0.0084f; // per mm (old 0.00008 per lPos)
            static constexpr float    kFrontLineKi          = 0.0f;
            static constexpr float    kFrontLineKd          = 0.0f;
            static constexpr float    kFrontLineDTauS       = 0.03f;
            static constexpr float    kFrontLineDeadbandMm  = 1.4f;    // old 150 lPos
            static constexpr float    kFrontLineSlewPerS    = 2.0f;    // output units per second
            static constexpr uint32_t kFrontLinePeriodMs    = 10;
            static constexpr uint32_t kFrontLostRecoverMs   = 300;     // push toward the lost side this long
            static constexpr float    kFrontLostEdgeMm      = 15.0f;   // only if the line was lost this far off center

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
            static constexpr bool     kFrontLineTeleplot = false; // qtrFront PD signals in Teleplot format, every control tick
        } // namespace Debug
    } // namespace StateMachine
} // namespace Constants
