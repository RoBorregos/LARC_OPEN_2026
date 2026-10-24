/**
 * @file I2cRecovery.hpp
 *
 * @brief Frees an I2C bus left mid-transaction by a reset (e.g. a firmware
 *        upload while the ToFs were being read): a slave can be left holding
 *        SDA low, so the next init writes garbage or fails. Clocks SCL until
 *        SDA is released, then sends a STOP. Call before Wire.begin().
 */

#pragma once

#include <Arduino.h>

// Returns false if SDA is still held low afterwards.
inline bool recoverI2cBus(uint8_t sdaPin, uint8_t sclPin)
{
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(10);

    for (uint8_t i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++)
    {
        pinMode(sclPin, OUTPUT);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(10);
        pinMode(sclPin, INPUT_PULLUP);
        delayMicroseconds(10);
    }

    // STOP: SDA rises while SCL is high
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(10);
    pinMode(sdaPin, INPUT_PULLUP);
    delayMicroseconds(10);

    return digitalRead(sdaPin) == HIGH;
}
