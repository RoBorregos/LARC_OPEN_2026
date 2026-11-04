/**
 * @file IR.cpp
 * @date 2026-02-10
 *
 * @brief Implementation of line IR sensors with per-sensor inversion
 */

#include "IR.hpp"
#include "constants.h"

// Constructor

IRLine::IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin,
               uint8_t invertedMask)
    : initialized(false),
      invertedMask(invertedMask),
      mux(nullptr)
{
    pins[FL] = flPin;
    pins[FR] = frPin;
    pins[BL] = blPin;
    pins[BR] = brPin;

    for (uint8_t i = 0; i < N; i++)
    {
        rawState[i]  = false;
        lineState[i] = false;
        pending[i] = false;
        useMux[i]    = false;
    }
}

IRLine::IRLine(const uint8_t pins_[N], uint8_t invertedMask)
    : initialized(false),
      invertedMask(invertedMask),
      mux(nullptr)
{
    for (uint8_t i = 0; i < N; i++)
    {
        pins[i]      = pins_[i];
        rawState[i]  = false;
        lineState[i] = false;
        pending[i] = false;
        useMux[i]    = false;
    }
}

IRLine::IRLine(uint8_t flPin, uint8_t frPin, Mux74HC4067& mux_,
               uint8_t blMuxCh, uint8_t brMuxCh, uint8_t invertedMask)
    : initialized(false),
      invertedMask(invertedMask),
      mux(&mux_)
{
    pins[FL] = flPin;
    pins[FR] = frPin;
    useMux[FL] = false;
    useMux[FR] = false;

    useMux[BL] = true;
    useMux[BR] = true;
    muxCh[BL]  = blMuxCh;
    muxCh[BR]  = brMuxCh;
    threshold[BL] = Constants::IRCalibration::kThreshBL;
    threshold[BR] = Constants::IRCalibration::kThreshBR;

    for (uint8_t i = 0; i < N; i++)
    {
        rawState[i]  = false;
        lineState[i] = false;
        pending[i] = false;
    }
}

// Initialization

bool IRLine::begin()
{
    for (uint8_t i = 0; i < N; i++)
    {
        if (!useMux[i])
            pinMode(pins[i], INPUT);
    }

    if (mux != nullptr)
        mux->begin();

    initialized = true;
    firstRead = true;
    update(); // First reading
    return true;
}

// Reading

void IRLine::update()
{
    if (!initialized)
        return;

    const uint16_t hyst = Constants::IRCalibration::kHysteresis;
    const uint32_t now = millis();

    for (uint8_t i = 0; i < N; i++)
    {
        bool raw = rawState[i];
        if (useMux[i])
        {
            const uint16_t value = mux->read(muxCh[i]);
            const uint16_t high = threshold[i] + hyst;
            const uint16_t low  = (threshold[i] > hyst) ? threshold[i] - hyst : 0;
            if (value >= high)     raw = true;
            else if (value <= low) raw = false;
        }
        else
        {
            raw = (digitalRead(pins[i]) == HIGH);
        }
        rawState[i] = raw;

        const bool isInverted = (invertedMask >> i) & 0x01;
        const bool candidate = isInverted ? (!raw) : raw;

        if (!firstRead && candidate != lineState[i])
        {
            if (!pending[i])
            {
                pending[i] = true;
                pendingSinceMs[i] = now;
            }
            const bool isBack = (i == BL || i == BR);
            const uint32_t neededMs = !candidate ? Constants::IRCalibration::kOffDebounceMs
                                    : isBack     ? Constants::IRCalibration::kBackOnDebounceMs
                                                 : Constants::IRCalibration::kOnDebounceMs;
            if ((now - pendingSinceMs[i]) < neededMs)
                continue;
        }
        lineState[i] = candidate;
        pending[i] = false;
    }
    firstRead = false;
}

// Getters

bool IRLine::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}