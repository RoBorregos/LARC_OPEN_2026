/**
 * @file qtr_test.cpp
 * @brief Diagnostico standalone (sin RTOS) del QTRX-MD-08A (array de
 *        reflectancia analogico, 8 canales) cableado a traves del mux
 *        74HC4067 (ver lib/sensors/mux.h e include/pins.h: kMuxSig /
 *        kMuxS0-S3, kQtrFrontFirstCh = canales 0..7 del mux).
 *
 *        Lee los 8 canales directo via Mux74HC4067::read() -- NO usa la
 *        clase QTR (lib/sensors/qtr) porque esta hardcodeada a N=7
 *        sensores, uno menos de los 8 que trae este array.
 *
 * Como usarlo: abre el monitor serie y pasa el array sobre blanco y negro.
 * Cada columna Cn debe subir/bajar de forma estable con la reflectancia
 * (mas reflectante = valor mas alto). Anota los min/max mostrados para
 * armar la calibracion despues (setCalibration en constants.h).
 *
 * pio run -e qtr_test -t upload -t monitor
 */

#include <Arduino.h>
#include "mux.h"

static Mux74HC4067 mux(Pins::kMuxSig);
static constexpr uint8_t kFirstCh = Pins::kQtrFrontFirstCh; // 0 (C0..C7)
static constexpr uint8_t kNumCh   = 8;

static uint16_t minVal[kNumCh];
static uint16_t maxVal[kNumCh];

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    mux.begin();

    for (uint8_t i = 0; i < kNumCh; ++i)
    {
        minVal[i] = 1023;
        maxVal[i] = 0;
    }

    Serial.println(F("[QTR TEST] QTRX-MD-08A (8 canales) via mux 74HC4067"));
    Serial.printf("[QTR TEST] SIG=%u S0=%u S1=%u S2=%u S3=%u, canales %u..%u\n",
                  Pins::kMuxSig, Pins::kMuxS0, Pins::kMuxS1, Pins::kMuxS2, Pins::kMuxS3,
                  kFirstCh, kFirstCh + kNumCh - 1);
}

void loop()
{
    uint16_t raw[kNumCh];

    for (uint8_t i = 0; i < kNumCh; ++i)
    {
        raw[i] = mux.read(kFirstCh + i);
        if (raw[i] < minVal[i]) minVal[i] = raw[i];
        if (raw[i] > maxVal[i]) maxVal[i] = raw[i];
    }

    static uint32_t lastPrint = 0;
    const uint32_t now = millis();
    if (now - lastPrint >= 100)
    {
        lastPrint = now;

        Serial.print(F("RAW: "));
        for (uint8_t i = 0; i < kNumCh; ++i)
            Serial.printf("C%u=%4u ", kFirstCh + i, raw[i]);

        Serial.print(F(" | min/max: "));
        for (uint8_t i = 0; i < kNumCh; ++i)
            Serial.printf("[%u,%u] ", minVal[i], maxVal[i]);

        Serial.println();
    }
}
