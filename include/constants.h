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
#include "config/DriveConfig.h"
#include "config/SensorConfig.h"
#include "config/StateMachineConfig.h"

namespace Constants
{
    namespace SystemConstants
    {
        constexpr float kUpdateInterval = 20.0;
    } // namespace SystemConstants

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

        // PROVISIONAL (missing mechanical tests)
        // test/servos/02_pca9685_channel_test.cpp before trusting them.
        static constexpr ServoCalib kCalib[SERVO_COUNT] = {
            { Pins::Servos::kIntakeUpperCh, 500, 2500, 50, 137 }, // INTAKE_UPPER
            { Pins::Servos::kIntakeLowerCh, 500, 2500, 50, 168 }, // INTAKE_LOWER
            { Pins::Servos::kSeparatorCh,   500, 2500, 58, 149 }, // SEPARATOR
            { Pins::Servos::kBenefit1Ch,    500, 2500, 0, 150 }, // BENEFIT_1
            { Pins::Servos::kBenefit2Ch,    500, 2500, 10, 160 }  // BENEFIT_2
        };

        // Positions (deg) 
        static constexpr uint8_t kIntakeUpperHome   = 50;
        static constexpr uint8_t kIntakeUpperDeploy = 110;

        static constexpr uint8_t kIntakeLowerHome   = 57;
        static constexpr uint8_t kIntakeLowerDeploy = 90;

        static constexpr uint8_t kSeparatorNeutral  = 101;
        static constexpr uint8_t kSeparatorLeft     = 57; // mature
        static constexpr uint8_t kSeparatorRight    = 148; // overmature

        static constexpr uint8_t kBenefit2Closed    = 153;
        static constexpr uint8_t kBenefit2Open      = 67;
        static constexpr uint8_t kBenefit1Closed    = 24;
        static constexpr uint8_t kBenefit1Open      = 120;

        // Time for benefit doors to stay open before closing automatically (ms)
        static constexpr uint32_t kBenefitOpenMs = 2500;

        // BENEFITS routine: how long the robot stays stopped at a box (ms).
        // The door opens kBenefitOpenMs of it.
        static constexpr uint32_t kBenefitStopMs = 3000;
        static_assert(kBenefitStopMs > kBenefitOpenMs, "kBenefitStopMs must be longer than kBenefitOpenMs");

        // BENEFITS routine: the same box colour must be seen this long while
        // stopped before the door opens (ms).
        static constexpr uint32_t kBenefitConfirmMs = 300;

        // BENEFITS routine: after a stop, the box must have FULLY left the
        // camera (no ROI sees it) this long while driving before the robot may
        // stop again (ms). Must be shorter than the gap between two boxes.
        static constexpr uint32_t kBenefitRearmMs = 400;

        // BENEFITS routine: the box must stay centred this long while driving
        // before the robot stops (ms). Filters a one-frame CENTER at the edge.
        static constexpr uint32_t kBenefitApproachMs = 100;

        // A servo write that failed on I2C is retried this often until it lands (ms).
        static constexpr uint32_t kServoRetryMs = 10;
    } // namespace ServoConfig

    namespace VisionConfig
    {
        // How many consecutive, fully valid, non-duplicate
        // frames must carry the SAME (phase, payload) before the Teensy
        // acts on it. 
        // 2 ignores single-frame flickers (ball bouncing in the ROI while moving), at the cost of one frame of delay.
        static constexpr uint8_t REQUIRED_CONFIRMATION_FRAMES = 2;

        // No valid new frame for this long, then every actuator goes safe 
        static constexpr uint32_t kLinkTimeoutMs = 500;

        // true: BENEFIT_1 bit from the Orin opens door 2 and vice versa (each box was getting the other's balls).
        static constexpr bool kSwapBenefitDoors = true;
        static constexpr uint32_t kSerialBaud = 115200;
    } // namespace VisionConfig

} // namespace Constants

#endif