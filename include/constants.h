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

#include "pins.h" // servo channels referenced by ServoConfig::kCalib

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

        constexpr Profile Front = {
            {76, 74, 69, 73, 158, 98, 75},          // On white
             {901, 892, 868, 870, 904, 908, 916}}; // black (up and DOWN)

        constexpr Profile Rear = {
             {68, 64, 66, 71, 73, 74, 304},
            {860, 843, 850, 939, 873, 874, 936}};

        constexpr uint16_t kBinaryThreshold = 600; // 0-1000 normalized, tune this one value

    } // namespace QTRCalibration

    namespace LineFollower

    {
        constexpr int kSetpoint = 2900;//2700; // center of 0-6000 range (el ultimo no se esta tomndo otherwise 0-7000)
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

    namespace ServoConfig
    {
        enum ServoIndex : uint8_t
        {
            INTAKE_UPPER = 0,
            INTAKE_LOWER = 1,
            SEPARATOR    = 2,
            BENEFIT_1    = 3,
            BENEFIT_2    = 4,
            SERVO_COUNT  = 5
        };

        // PCA9685 timing
        // Generic boards often run the oscillator at 26-27 MHz, not 25.
        // If a 1500 us command measures e.g. 1560 us on a scope, set this
        // to 25000000 * 1560 / 1500 = 26000000.
        static constexpr uint32_t kPcaOscillatorHz = 25000000;
        static constexpr float    kServoPwmFreqHz  = 50.0f;
        struct ServoCalib
        {
            uint8_t  channel;
            uint16_t minPulseUs;
            uint16_t maxPulseUs;
            uint8_t  minAngleDeg;
            uint8_t  maxAngleDeg;
        };

        // PROVISIONAL pulse limits verify each servo with
        // test/servos/02_pca9685_channel_test.cpp before trusting them.
        static constexpr ServoCalib kCalib[SERVO_COUNT] = {
            { Pins::Servos::kIntakeUpperCh, 500, 2500, 40, 100 }, // INTAKE_UPPER
            { Pins::Servos::kIntakeLowerCh, 500, 2500, 40, 100 }, // INTAKE_LOWER
            { Pins::Servos::kSeparatorCh,   500, 2500, 61, 149 }, // SEPARATOR
            { Pins::Servos::kBenefit1Ch,    500, 2500, 20, 160 }, // BENEFIT_1
            { Pins::Servos::kBenefit2Ch,    500, 2500, 20, 160 }  // BENEFIT_2
        };

        // Positions (deg) 
        static constexpr uint8_t kIntakeUpperHome   = 50;
        static constexpr uint8_t kIntakeUpperDeploy = 85;

        static constexpr uint8_t kIntakeLowerHome   = 50;
        static constexpr uint8_t kIntakeLowerDeploy = 80;

        static constexpr uint8_t kSeparatorNeutral  = 101;
        static constexpr uint8_t kSeparatorLeft     = 66; // mature
        static constexpr uint8_t kSeparatorRight    = 142; // immature

        static constexpr uint8_t kBenefit1Closed    = 90;
        static constexpr uint8_t kBenefit1Open      = 152;
        static constexpr uint8_t kBenefit2Closed    = 90;
        static constexpr uint8_t kBenefit2Open      = 30;

        // Time for benefit doors to stay open before closing automatically (ms)
        static constexpr uint32_t kBenefitOpenMs = 600;
    } // namespace ServoConfig

    namespace VisionConfig
    {
        // How many consecutive, fully valid, non-duplicate
        // frames must carry the SAME (phase, payload) before the Teensy
        // acts on it. 
        static constexpr uint8_t REQUIRED_CONFIRMATION_FRAMES = 1;

        // No valid new frame for this long, then every actuator goes safe and
        static constexpr uint32_t kLinkTimeoutMs = 500;
        static constexpr uint32_t kSerialBaud = 115200;
    } // namespace VisionConfig

    namespace ToFConfig
    {
        static constexpr uint16_t kTimeoutMs = 100;
        static constexpr uint32_t kTimingBudgetUs = 20000;
        static constexpr uint16_t kContinuousPeriodMs = 20;
        static constexpr uint16_t kMuxSettleDelayMs = 10;
    } //

} // namespace Constants

#endif