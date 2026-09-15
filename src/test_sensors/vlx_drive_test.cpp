/**
 * @file vlx_drive_test.cpp
 * @brief Combina los 4 ToF (VL53L1X) detras del TCA9548A en Wire1 (canales
 *        0/1/3/4, ver vlx_single_test.cpp -- que solo cubria 3 y 4) con la
 *        clase Drive completa: mientras cualquiera detecte algo mas cerca
 *        que kDetectMm, el robot se desplaza a la izquierda; en cuanto
 *        ninguno detecta, retoma el avance hacia el frente (se re-evalua en
 *        cada ronda, sin estado).
 *
 *        Posicion fisica por canal (confirmar en campo si difiere):
 *        canal 0 = FL, canal 1 = FR, canal 3 = BL, canal 4 = BR.
 *
 * pio run -e vlx_drive_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

#include "pins.h"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive drive;

constexpr uint8_t kTcaAddress = 0x70;
constexpr int16_t kDetectMm   = 600;   // umbral de deteccion, ajustar en campo
constexpr float   kRightSpeed = 0.35f; // velocidad lateral, ajustar en campo

Adafruit_VL53L1X vlxFL = Adafruit_VL53L1X(); // canal 0
Adafruit_VL53L1X vlxFR = Adafruit_VL53L1X(); // canal 1
Adafruit_VL53L1X vlxBL = Adafruit_VL53L1X(); // canal 3
Adafruit_VL53L1X vlxBR = Adafruit_VL53L1X(); // canal 4

struct ToFSlot
{
    const char*      name;
    uint8_t          channel;
    Adafruit_VL53L1X* sensor;
    bool             ok;
};

ToFSlot tofs[] = {
    {"FL", 0, &vlxFL, false},
    {"FR", 1, &vlxFR, false},
    {"BL", 3, &vlxBL, false},
    {"BR", 4, &vlxBR, false},
};

constexpr uint8_t kNumToF = sizeof(tofs) / sizeof(tofs[0]);

void tcaSelect(uint8_t channel)
{
    if (channel > 7) return;

    Wire1.beginTransmission(kTcaAddress);
    Wire1.write(1 << channel);
    Wire1.endTransmission();

    delay(5);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire1.begin();
    Wire1.setClock(100000);

    Serial.println("=== Drive + VLX (canales 0/1/3/4 = FL/FR/BL/BR) ===");

    for (uint8_t i = 0; i < kNumToF; i++)
    {
        ToFSlot& slot = tofs[i];
        tcaSelect(slot.channel);

        if (!slot.sensor->begin(0x29, &Wire1))
        {
            Serial.printf("ERROR inicializando VL53L1X %s (canal %u)\n", slot.name, slot.channel);
            continue;
        }

        slot.ok = slot.sensor->startRanging();
        Serial.printf("VL53L1X %s (canal %u): %s\n", slot.name, slot.channel,
                      slot.ok ? "OK" : "ERROR iniciando ranging");
    }

    drive.begin();
    drive.holdYaw(true);
    drive.setTargetYaw(drive.getYaw());
}

void loop()
{
    uint32_t roundStart = millis();

    bool anyDetect = false;

    for (uint8_t i = 0; i < kNumToF; i++)
    {
        ToFSlot& slot = tofs[i];
        tcaSelect(slot.channel);

        Serial.print(slot.name);
        Serial.print(": ");

        int16_t distanciaMm = -1;
        if (slot.ok && slot.sensor->dataReady())
        {
            distanciaMm = slot.sensor->distance();

            if (distanciaMm == -1)
            {
                Serial.print("ERROR");
            }
            else
            {
                Serial.print(distanciaMm);
                Serial.print(" mm");
            }

            slot.sensor->clearInterrupt();
        }
        else
        {
            Serial.print(slot.ok ? "esperando..." : "sensor no inicializado");
        }

        Serial.print("   |   ");

        if (distanciaMm != -1 && distanciaMm < kDetectMm)
            anyDetect = true;
    }

    // =========================
    // Decision de movimiento
    // =========================
    if (anyDetect)
    {
        drive.left(kRightSpeed);
        Serial.println("   ->  IZQUIERDA");
    }
    else
    {
        drive.forward(kRightSpeed);
        Serial.println("   ->  FORWARD");
    }

    // Igual que vlx_single_test.cpp: da ~100ms de margen entre rondas de
    // lectura ToF, pero sin bloquear el control de Drive (BNO + yaw PID +
    // motores), que necesita seguir corriendo cada 10ms.
    while (millis() - roundStart < 100)
    {
        drive.update();
    }
}
