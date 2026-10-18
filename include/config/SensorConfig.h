/**
 * @file SensorConfig.h
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Sensor constants: QTR, IR, ultrasonic, ToF and Perception.
 */

#pragma once

#include <Arduino.h>

namespace Constants
{
    namespace QTRCalibration
    {
        constexpr size_t kNumSensors = 8;

        struct Profile
        {
            uint16_t min[kNumSensors];
            uint16_t max[kNumSensors];
        };

        constexpr Profile Front = {
             {69, 66, 66, 69, 93, 80, 76},          // On white
             {859, 873, 853, 868, 884, 876, 874}}; // black (up and DOWN)

        constexpr Profile Rear = {
              {73, 76, 106, 167, 250, 166},
            {842, 880, 918, 946, 897, 897}};

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

    namespace UltrasonicConstants
    {
        static constexpr uint32_t kPingPeriodMs = 50;     // ms entre pings
        static constexpr uint32_t kTrigHighUs = 10;       // µs que el trigger está en HIGH
        static constexpr uint32_t kEchoTimeoutUs = 25000; // µs (~4 m máx)
    } // namespace UltrasonicConstants

    namespace ToFConfig
    {
        static constexpr uint16_t kTimeoutMs = 100;
        static constexpr uint32_t kTimingBudgetUs = 20000;
        static constexpr uint16_t kContinuousPeriodMs = 20;
        static constexpr uint16_t kMuxSettleDelayMs = 10;
    } // namespace ToFConfig

    namespace PerceptionConfig
    {
        static constexpr uint16_t kTofMaxRangeMm      = 1000;
        static constexpr uint16_t kObstacleDistanceMm = 215;
        static constexpr uint32_t kTofWarmupMs        = 500;

        // Obstacle latch (front UL/UR -> POOL, rear LL/LR -> POOLSGOBACK)
        static constexpr uint32_t kObstacleReleaseMs  = 400;
        static constexpr uint32_t kObstacleConfirmMs  = 0;
    } // namespace PerceptionConfig
} // namespace Constants
