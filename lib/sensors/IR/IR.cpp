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
    update(); // First reading
    return true;
}

// Reading

void IRLine::update()
{
    if (!initialized)
        return;

    for (uint8_t i = 0; i < N; i++)
    {
        bool raw;
        if (useMux[i])
            raw = (mux->read(muxCh[i]) >= threshold[i]); // analog + umbral
        else
            raw = (digitalRead(pins[i]) == HIGH); // HIGH true

        rawState[i] = raw;

        const bool isInverted = (invertedMask >> i) & 0x01;
        lineState[i] = isInverted ? (!raw) : raw;
    }
}

// Getters

bool IRLine::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}