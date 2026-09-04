/**
 * @file constants.h
 * @date 12/01/2026
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Constants for the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <math.h>

namespace Constants
{
    namespace SystemConstants
    {
        constexpr float kUpdateInterval = 20.0;
    } // namespace SystemConstants

    namespace Kinematics
    {
        // "omni_motors" class
        constexpr float M1_ANGLE = 135.0f;  // M1     UL
        constexpr float M2_ANGLE = 45.0f;   // M2     UR
        constexpr float M3_ANGLE = -135.0f; // M3     LL
        constexpr float M4_ANGLE = -45.0f;  // M4     LR
    } // namespace Kinematics

    namespace DriveConstants
    {
        static constexpr float kDEG2RAD = PI / 180.0f;

        constexpr float kWheelDiameter = 0.109f;
        constexpr float kWheelRaius = kWheelDiameter / 2.0;
        constexpr float kWheelCircumference = 2 * M_PI * kWheelRaius;
    } // namespace DriveConstants
    namespace PID
    {
        static constexpr float kKp = 1.20f;//1.20;   // 4.8f;  2.5f; 1.5f
        static constexpr float kKi = 0.002f; // 0.002f
        static constexpr float kKd = 0.0042f;  // 0.0012f; // 0.06f; 0.0f
        static constexpr float kOmegaMax = 0.25f; //0.25f;
        static constexpr float kcurrentVelocity = 0.30f; // Velocity according to PID

    } // namespace PID
    namespace UltrasonicConstants
    {
        static constexpr uint32_t kPingPeriodMs = 50;     // ms entre pings
        static constexpr uint32_t kTrigHighUs = 10;       // µs que el trigger está en HIGH
        static constexpr uint32_t kEchoTimeoutUs = 25000; // µs (~4 m máx)
    } // namespace UltrasonicConstants

    namespace QTRCalibration
    {
        constexpr size_t kNumSensors = 8; 

        struct Profile
        {
            uint16_t min[kNumSensors];


            uint16_t max[kNumSensors];
        };

        // PLACEHOLDERS
        constexpr Profile Front = {
            {811, 805, 846, 841, 856, 855, 869},          // On white
            {997, 995, 1001, 1001, 1001, 1000, 999}}; // black (up and DOWN)

        constexpr Profile Rear = {
            {120, 130, 115, 140, 150, 135, 128, 122},
            {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000}};

        constexpr uint16_t kBinaryThreshold = 600; // 0-1000 normalized, tune this one value

    } // namespace QTRCalibration

    namespace LineFollower

    {
        constexpr int kSetpoint = 2700;//2700; // center of 0-6000 range (el ultimo no se esta tomndo otherwise 0-7000)
        // change to 4000, 2000, etc. as needed
    }

    namespace IRCalibration
    {
        // Umbrales analógicos por sensor (0..1023).
        // Si raw >= umbral = línea detectada (antes de aplicar inversión).

        // PLACEHOLDERS
        static constexpr uint16_t kThreshFL = 50; // Front-Left
        static constexpr uint16_t kThreshFR = 300; // Front-Right
        static constexpr uint16_t kThreshBL = 50;  // Back-Left
        static constexpr uint16_t kThreshBR = 112; // Back-Right

        static constexpr uint16_t kHysteresis = 5;    // Hysteresis margin (same for all sensors)
        static constexpr uint16_t kDebounceCount = 3; // Number of consecutive readings to confirm state change
    } // namespace IRCalibration

    namespace ServoAngles
    {
       // ── Startup ramp ────────────────────────────────────────
        // Speed (deg/s) for the blocking ramp in begin()
        static constexpr float kStartupSpeed = 70.0f;

        // Where attach() physically puts each servo (measure per servo)
        static constexpr uint8_t kMechStartIntakeUpper = 154;
        static constexpr uint8_t kMechStartIntakeLower = 160;
        static constexpr uint8_t kMechStartSeparator   = 90;
        static constexpr uint8_t kMechStartBenefit     = 90;
        static constexpr uint8_t kMechStartHolder      = 90;

        // Servo positions
        static constexpr uint8_t kIntakeUpperHome   = 50;
        static constexpr uint8_t kIntakeUpperDeploy = 85;

        static constexpr uint8_t kIntakeLowerHome   = 50;
        static constexpr uint8_t kIntakeLowerDeploy = 80;

        static constexpr uint8_t kSeparatorCenter = 70;
        static constexpr uint8_t kSeparatorLeft   = 100;
        static constexpr uint8_t kSeparatorRight  = 120;

        static constexpr uint8_t kBenefitRed    = 70;
        static constexpr uint8_t kBenefitCenter = 90;
        static constexpr uint8_t kBenefitBlue   = 110;

        static constexpr uint8_t kHolderHold    = 70;
        static constexpr uint8_t kHolderRelease = 50;

        // ── Busy time (ms) — isBusy() returns true for this long after a move
        static constexpr uint32_t kIntakeUpperMoveMs = 200;
        static constexpr uint32_t kIntakeLowerMoveMs = 200;
        static constexpr uint32_t kSeparatorMoveMs   = 50;
        static constexpr uint32_t kBenefitMoveMs     = 50;
        static constexpr uint32_t kHolderMoveMs      = 50;

        // ── Detach time (ms) — PWM cut after this long with no movement
        static constexpr uint32_t kIntakeUpperDetachMs = 10000;
        static constexpr uint32_t kIntakeLowerDetachMs = 10000;
        static constexpr uint32_t kSeparatorDetachMs   = 50;
        static constexpr uint32_t kBenefitDetachMs     = 50;
        static constexpr uint32_t kHolderDetachMs      = 50;

        // Debounce / hold (used by separatorUpdate / intakeUpdate) ──
        // Separator: consecutive calls needed before actually moving
        static constexpr uint8_t  kSepConfirm = 2;
        // Separator: ms of no detection before returning to center
        static constexpr uint32_t kSepClearMs = 300;
        // Intakes: ms to hold deployed after last detection
        static constexpr uint32_t kIntakeHoldMs = 100;

    } // namespace ServoAngles

    namespace ToFConfig
    {
        static constexpr uint16_t kTimeoutMs = 100;
        static constexpr uint32_t kTimingBudgetUs = 20000;
        static constexpr uint16_t kContinuousPeriodMs = 20;
        static constexpr uint16_t kMuxSettleDelayMs = 10;
    } //

} // namespace Constants

#endif