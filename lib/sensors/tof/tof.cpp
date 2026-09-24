/*
 * @file tof.cpp
 * @date 2026-01-28
 *
 * @brief VL53L0X / VL53L1X TOF distance sensor (I2C)
 *        Millis-based update with last-known good reading.
*/

#include "tof.hpp"

ToF::ToF()
    : type_(ToFType::L1X),
      initialized(false), continuous(false),
      useMux(false),
      distanceMm(INVALID_MM),
      lastUpdateMs_(0),
      updateIntervalMs_(Constants::ToFConfig::kContinuousPeriodMs),
      maxRangeMm_(600),
      muxChannel_(0), mux_(nullptr)
{}

ToF::ToF(uint8_t muxChannel, TCA9548A& mux, ToFType type)
    : type_(type),
      initialized(false), continuous(false),
      useMux(true),
      distanceMm(INVALID_MM),
      lastUpdateMs_(0),
      updateIntervalMs_(Constants::ToFConfig::kContinuousPeriodMs),
      maxRangeMm_(600),
      muxChannel_(muxChannel), mux_(&mux)
{}

bool ToF::begin()
{
    // Detras del mux el sensor esta en el mismo bus que el TCA (Wire1 en
    // el robot actual). Pololu (L0X) usa &Wire por default.
    sensorL0X.setBus(bus());

    selectIfMux();
    
    // Non blocking settle: just record time; first update() will
    // respect the interval guard anyway.
    uint32_t settleStart = millis();
    while (millis() - settleStart < Constants::ToFConfig::kMuxSettleDelayMs);

    bool ok = false;

    if (type_ == ToFType::L0X) {
        sensorL0X.setTimeout(Constants::ToFConfig::kTimeoutMs);
        ok = sensorL0X.init();
        if (ok) {
            sensorL0X.setMeasurementTimingBudget(Constants::ToFConfig::kTimingBudgetUs);
            sensorL0X.startContinuous(Constants::ToFConfig::kContinuousPeriodMs);
        }
    } else {
        // Igual que vlx_single_test.cpp: begin(0x29, bus) + startRanging()
        // con la configuracion default de la libreria.
        ok = sensorL1X.begin(0x29, bus()) && sensorL1X.startRanging();
    }

    if (!ok) {
        initialized = false;
        return false;
    }

    continuous    = true;
    initialized   = true;
    lastUpdateMs_ = millis();

    return true;
}

void ToF::update()
{
    if (!initialized) return;

    // Rate limit: don't poll faster than the sensor produces data
    uint32_t now = millis();
    if (now - lastUpdateMs_ < updateIntervalMs_) return;
    lastUpdateMs_ = now;

    selectIfMux();

    if (type_ == ToFType::L0X) {
        // Single read call. avoids the double read bug
        uint16_t raw = sensorL0X.readRangeContinuousMillimeters();

        if (sensorL0X.timeoutOccurred()) {
            // Keep last known good value, don't overwrite with INVALID
            return;
        }
        if (raw == 65535 || raw == 0) {
            // Sensor returned garbage = keep last value
            return;
        }
        if (raw > maxRangeMm_) {
            distanceMm = maxRangeMm_; // valor alto = sin obstáculo
            return;
        }
        distanceMm = raw;

    } else {
        if (!sensorL1X.dataReady()) {
            // No new measurement yet = keep last value
            return;
        }

        // Igual que vlx_single_test.cpp: se lee status + distancia directo
        // (Adafruit distance() regresa -1 para cualquier status != 0).
        uint8_t  status = 0;
        uint16_t raw    = 0;
        const bool i2cOk = sensorL1X.VL53L1X_GetRangeStatus(&status) == 0 &&
                           sensorL1X.VL53L1X_GetDistance(&raw) == 0;
        sensorL1X.clearInterrupt();

        if (!i2cOk) return; // keep last value

        // Con status invalido el raw es basura (ej. wraparound reporta
        // ~300 mm cuando no hay nada en ~4 m), asi que no se usa como
        // distancia. Codigos del ULD de ST: 0 OK, 1 sigma, 2 signal,
        // 4 fuera de rango, 5 hardware, 7 wraparound.
        switch (status) {
            case 0:
                break;
            case 2:
            case 4:
            case 7:
                distanceMm = maxRangeMm_; // nada cerca
                return;
            default:
                return; // lectura dudosa = keep last value
        }

        if (raw == 0) return;
        if (raw > maxRangeMm_) {
            distanceMm = maxRangeMm_; // valor alto = sin obstáculo
            return;
        }
        distanceMm = raw;
    }
}

void ToF::setMaxRange(uint16_t mm)
{
    maxRangeMm_ = mm;
}

void ToF::setUpdateInterval(uint16_t ms)
{
    updateIntervalMs_ = ms;
}

void ToF::setTimingBudgetMs(uint16_t ms)
{
    if (!initialized) return;
    selectIfMux();

    uint32_t budgetUs = (uint32_t)ms * 1000UL;

    if (type_ == ToFType::L0X)
        sensorL0X.setMeasurementTimingBudget(budgetUs);
    else
        sensorL1X.setTimingBudget(ms);
}

void ToF::setInterMeasurementMs(uint16_t ms)
{
    if (!initialized) return;
    selectIfMux();
    if (!continuous) return;

    if (type_ == ToFType::L0X)
        sensorL0X.startContinuous(ms);
    else
        sensorL1X.VL53L1X_SetInterMeasurementInMs(ms);

    // Keep the software rate limit in sync
    updateIntervalMs_ = ms;
}

void ToF::startContinuous(uint16_t periodMs)
{
    if (!initialized) return;
    selectIfMux();

    if (type_ == ToFType::L0X)
        sensorL0X.startContinuous(periodMs);
    else {
        sensorL1X.VL53L1X_SetInterMeasurementInMs(periodMs);
        sensorL1X.startRanging();
    }

    continuous       = true;
    updateIntervalMs_ = periodMs;
}

void ToF::stopContinuous()
{
    if (!initialized) return;
    selectIfMux();

    if (type_ == ToFType::L0X)
        sensorL0X.stopContinuous();
    else
        sensorL1X.stopRanging();

    continuous = false;
}