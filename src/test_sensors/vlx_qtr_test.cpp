/**
 * @file vlx_qtr_test.cpp
 * @brief Diagnostico standalone (sin motores/BNO/IR): lee los 4 sensores
 *        ToF (VLX, VL53L1X) via TCA9548A y escanea los 16 canales del
 *        segundo mux 74HC4067 (Pins::kMuxSig2 / net SIG_A1, pin 22 segun
 *        esquematico) donde esta conectado el QTR que se esta probando.
 *
 *        ToF: FR/FL/BL/BR = canales 0/1/2/3 del TCA9548A (ver pins.h).
 *        Solo el canal 3 (BR) tiene un VLX confirmado conectado por ahora
 *        -- los otros 3 son placeholders y van a reportar FAIL de init
 *        hasta que se conecten.
 *
 *        QTR: todavia no se sabe en que canal(es) del mux2 esta el QTR
 *        fisico, asi que en vez de asumir un rango fijo (como qtr_test.cpp
 *        con C0-C7 del mux1) se leen los 16 canales (C0..C15) del mux2
 *        para identificarlos a mano tapando/destapando cada sensor.
 *
 * pio run -e vlx_qtr_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "mux.h"
#include "TCA9548A/TCA9548A.h"
#include "tof/tof.hpp"

TCA9548A i2cMux;

// ToF (VLX) -- FR/FL/BL/BR por posicion fisica (ver pins.h kToFch*).
// Solo BR (canal 3) confirmado conectado; FR/FL/BL son placeholders.
ToF tofFR(Pins::kToFchFR, i2cMux, ToFType::L1X); // placeholder -- confirmar cableado
ToF tofFL(Pins::kToFchFL, i2cMux, ToFType::L1X); // placeholder -- confirmar cableado
ToF tofBL(Pins::kToFchBL, i2cMux, ToFType::L1X); // placeholder -- confirmar cableado
ToF tofBR(Pins::kToFchBR, i2cMux, ToFType::L1X); // confirmado: VLX conectado aqui

struct ToFSlot
{
    const char* name;
    ToF*        sensor;
};

ToFSlot tofs[] = {
    {"FR", &tofFR},
    {"FL", &tofFL},
    {"BL", &tofBL},
    {"BR", &tofBR},
};

constexpr uint8_t NUM_TOF = sizeof(tofs) / sizeof(tofs[0]);

// QTR fisico conectado al segundo mux 74HC4067 (SIG = Pins::kMuxSig2,
// select lines S0-S3 compartidas con el mux1) -- canal(es) exactos TBD.
Mux74HC4067 qtrMux(Pins::kMuxSig2);
constexpr uint8_t kNumMuxCh = 16;

uint16_t minVal[kNumMuxCh];
uint16_t maxVal[kNumMuxCh];

constexpr uint32_t kPrintPeriodMs = 150;

void printToF()
{
    Serial.print(F("VLX "));
    for (uint8_t i = 0; i < NUM_TOF; i++)
    {
        tofs[i].sensor->update();
        Serial.print(tofs[i].name);
        Serial.print(':');
        Serial.print(tofs[i].sensor->isValid() ? tofs[i].sensor->getDistanceCm() : -1.0f, 0);
        Serial.print(F("cm "));
    }
}

void printQtrScan()
{
    Serial.print(F("| QTR raw: "));
    for (uint8_t ch = 0; ch < kNumMuxCh; ch++)
    {
        uint16_t v = qtrMux.read(ch);
        if (v < minVal[ch]) minVal[ch] = v;
        if (v > maxVal[ch]) maxVal[ch] = v;
        Serial.printf("C%u=%4u ", ch, v);
    }

    Serial.print(F("| min/max: "));
    for (uint8_t ch = 0; ch < kNumMuxCh; ch++)
        Serial.printf("[%u,%u] ", minVal[ch], maxVal[ch]);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("  VLX + QTR TEST (sin motores/BNO)");
    Serial.println("================================");

    // ToF (VLX) via TCA9548A
    Wire.begin();
    Wire.setClock(400000);
    i2cMux.begin();

    for (uint8_t i = 0; i < NUM_TOF; i++)
    {
        const bool ok = tofs[i].sensor->begin();
        tofs[i].sensor->setMaxRange(600);
        tofs[i].sensor->setUpdateInterval(30);
        Serial.print("tof "); Serial.print(tofs[i].name);
        Serial.print(" init: "); Serial.println(ok ? "OK" : "FAIL");
    }

    // QTR -- segundo mux 74HC4067 (SIG_A1)
    qtrMux.begin();
    for (uint8_t ch = 0; ch < kNumMuxCh; ch++)
    {
        minVal[ch] = 1023;
        maxVal[ch] = 0;
    }
    Serial.printf("QTR mux: SIG=%u (kMuxSig2) S0=%u S1=%u S2=%u S3=%u, canales 0..15\n",
                  Pins::kMuxSig2, Pins::kMuxS0, Pins::kMuxS1, Pins::kMuxS2, Pins::kMuxS3);

    Serial.println("Listo.");
    Serial.println();
}

void loop()
{
    static uint32_t lastPrintMs = 0;
    const uint32_t now = millis();
    if ((now - lastPrintMs) >= kPrintPeriodMs)
    {
        lastPrintMs = now;
        printToF();
        printQtrScan();
        Serial.println();
    }
}
