/**
 * @file tof.hpp
 * @date 2026-01-28
 *
 * @brief VL53L0X / VL53L1X TOF distance sensor (I2C)
 *        Millis-based update with last-known good reading.
 */

#ifndef TOF_HPP
#define TOF_HPP

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
// L1X via Adafruit_VL53L1X (mismo patron que vlx_single_test.cpp,
// confirmado en hardware). No incluir tambien <VL53L1X.h> de Pololu:
// ambas librerias definen una clase VL53L1X y chocan.
#include <Adafruit_VL53L1X.h>
#include "TCA9548A/TCA9548A.h"
#include "constants.h"

enum class ToFType : uint8_t {
    L0X,
    L1X
};

class ToF
{
public:
    static constexpr uint16_t INVALID_MM = 0xFFFF;

    ToF();
    ToF(uint8_t muxChannel, TCA9548A& mux, ToFType type = ToFType::L0X);

    bool begin();
    void update();

    uint16_t getDistanceMm()  const { return distanceMm; }
    bool     isInitialized()  const { return initialized; }

    /// Set the max useful range (readings beyond this are discarded).
    /// Default in constants.  Call before or after begin().
    void setMaxRange(uint16_t mm);

    /// Set the minimum interval between polls (ms).
    /// Defaults to kContinuousPeriodMs.  Automatically synced when
    /// you call setInterMeasurementMs() or startContinuous().
    void setUpdateInterval(uint16_t ms);

    void setTimingBudgetMs(uint16_t ms);
    void setInterMeasurementMs(uint16_t ms);
    void startContinuous(uint16_t periodMs = Constants::ToFConfig::kContinuousPeriodMs);
    void stopContinuous();

    bool isValid() const { 
    return initialized && distanceMm != INVALID_MM; 
    }

    float getDistanceCm() const { 
    return isValid() ? distanceMm / 10.0f : -1.0f; 
    }

    uint16_t getMaxRange() const { return maxRangeMm_; }

private:
    VL53L0X  sensorL0X;
    Adafruit_VL53L1X sensorL1X;
    ToFType  type_;

    bool     initialized;
    bool     continuous;
    bool     useMux;

    uint16_t distanceMm;
    uint32_t lastUpdateMs_;
    uint16_t updateIntervalMs_;
    uint16_t maxRangeMm_;


    uint8_t   muxChannel_;
    TCA9548A* mux_;

    TwoWire* bus() const { return (useMux && mux_) ? &mux_->wire() : &Wire; }

    inline void selectIfMux() {
        if (useMux && mux_) mux_->selectChannel(muxChannel_);
    }
};

#endif // TOF_HPP