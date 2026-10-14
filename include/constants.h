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
        static constexpr float kKd = 0.04f;  // 0.0042f; 0.0012f; 0.06f; 0.0f
        static constexpr float kOmegaMax = 0.25f; //0.25f;
        static constexpr float kOmegaMin = 0.12f;       // minimum omega that moves the motors (deadband)
        static constexpr float kYawDeadbandDeg = 1.5f;  // below this yaw error no minimum push
        static constexpr float kYawRateClosingDegS = 5.0f; // turning toward the target faster than this: no minimum push
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
             {66, 67, 68, 67, 75, 83, 72},          // On white
             {827, 869, 863, 863, 872, 880, 856}}; // black (up and DOWN)

        constexpr Profile Rear = {
              {68, 68, 78, 263, 256, 176},
            {824, 865, 888, 938, 892, 904}};

        constexpr uint16_t kBinaryThreshold = 600; // 0-1000 normalized, tune this one value

        // ── Autocalibracion (ver QTR::ambientCalibrate / beginAutoCal) ──────
        // Paso 1, ambient (robot quieto sobre fondo sin linea, al arrancar):
        constexpr uint16_t kAmbientMaxDelta   = 100; // max corrimiento global de calMin (cuentas ADC)
        constexpr uint16_t kAmbientRejectNorm = 200; // 0-1000: si algun sensor lee mas, NO esta sobre fondo -> se rechaza

        // Paso 2, ventana de aprendizaje por sensor (dentro de los estados):
        constexpr uint16_t kLearnBgNorm       = 150; // norm < esto -> lectura de fondo (alimenta calMin)
        constexpr uint16_t kLearnLineNorm     = 400; // norm > esto -> candidato a linea (alimenta calMax)
        constexpr uint8_t  kLearnHighSamples  = 3;   // lecturas seguidas altas antes de aceptar un maximo (rechaza picos EMI)
        constexpr float    kLearnBgAlpha      = 0.05f; // filtro exponencial del fondo
        constexpr uint16_t kLearnMinBand      = 120; // el min aprendido no se aleja mas de esto del valor previo
        constexpr uint16_t kLearnMaxBand      = 200; // idem para el max
        constexpr uint16_t kMinSpan           = 300; // rango minimo max-min; si no se cumple, el sensor no se actualiza

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
        static constexpr uint16_t kThreshBL = 112; // Back-Left  (mux C7)
        static constexpr uint16_t kThreshBR = 50;  // Back-Right (mux C15)

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
            { Pins::Servos::kIntakeUpperCh, 500, 2500, 85, 137 }, // INTAKE_UPPER
            { Pins::Servos::kIntakeLowerCh, 500, 2500, 65, 168 }, // INTAKE_LOWER
            { Pins::Servos::kSeparatorCh,   500, 2500, 58, 149 }, // SEPARATOR
            { Pins::Servos::kBenefit1Ch,    500, 2500, 0, 150 }, // BENEFIT_1
            { Pins::Servos::kBenefit2Ch,    500, 2500, 10, 160 }  // BENEFIT_2
        };

        // Positions (deg) 
        static constexpr uint8_t kIntakeUpperHome   = 97;
        static constexpr uint8_t kIntakeUpperDeploy = 133;

        static constexpr uint8_t kIntakeLowerHome   = 69;
        static constexpr uint8_t kIntakeLowerDeploy = 110;

        static constexpr uint8_t kSeparatorNeutral  = 108;
        static constexpr uint8_t kSeparatorLeft     = 57; // mature
        static constexpr uint8_t kSeparatorRight    = 142; // overmature

        static constexpr uint8_t kBenefit2Closed    = 153;
        static constexpr uint8_t kBenefit2Open      = 67;
        static constexpr uint8_t kBenefit1Closed    = 24;
        static constexpr uint8_t kBenefit1Open      = 107;

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
    } // namespace ServoConfig

    namespace VisionConfig
    {
        // How many consecutive, fully valid, non-duplicate
        // frames must carry the SAME (phase, payload) before the Teensy
        // acts on it. 
        static constexpr uint8_t REQUIRED_CONFIRMATION_FRAMES = 1; // (unchanged, good readings but not fully tested)

        // No valid new frame for this long, then every actuator goes safe 
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