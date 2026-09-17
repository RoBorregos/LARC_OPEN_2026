#pragma once

/**
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Decalration of line IR sensors with per-sensor inversion
 */

#include <Arduino.h>
#include "mux.h"

class IRLine
{
public:
    // Sensor Index
    enum Sensor : uint8_t
    {
        FL = 0, // Front-Left
        FR = 1, // Front-Right
        BL = 2, // Back-Left
        BR = 3, // Back-Right
        N  = 4
    };

    /**
     * Sensor Bitmask
     * Bit 0:FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
     * Ejemplo: 0b0011 invierte FL y FR.
     * By default 0b1111 (all inverted).
     */
    IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin,
           uint8_t invertedMask = 0b1111);

    // Alternate constructor with array of pins
    IRLine(const uint8_t pins_[N], uint8_t invertedMask = 0b1111);

    // Hybrid constructor: FL/FR se leen por GPIO directo (digitalRead),
    // BL/BR se leen a traves de un mux 74HC4067 compartido (analogRead +
    // umbral, ver Constants::IRCalibration::kThreshBL/kThreshBR).
    IRLine(uint8_t flPin, uint8_t frPin, Mux74HC4067& mux,
           uint8_t blMuxCh, uint8_t brMuxCh, uint8_t invertedMask = 0b1111);

    // Configs pins and does first reading
    bool begin();

    // Reads raw values of all sensors
    void update();

    // Returns the logic reading of sensors (after palying inversion)
    // true = Line
    // false = No line
    bool getState(Sensor s) const;

private:
    bool    initialized;
    uint8_t pins[N];
    bool    rawState[N];
    bool    lineState[N];
    uint8_t invertedMask;

    // Solo usados por los sensores en modo mux (useMux[i] == true).
    Mux74HC4067* mux;
    uint8_t      muxCh[N];
    uint16_t     threshold[N];
    bool         useMux[N];
};