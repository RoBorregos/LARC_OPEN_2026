/**
 * @file vlx_drive_test.cpp
 * @brief Combina los dos ToF (VL53L1X) detras del TCA9548A en Wire1 (canales
 *        3 y 4, ver vlx_single_test.cpp) con la clase Drive completa: mientras
 *        el canal 4 detecte algo mas cerca que kDetectMm, el robot se
 *        desplaza a la izquierda; en cuanto deja de detectar, retoma el
 *        avance hacia el frente (se re-evalua en cada ronda, sin estado).
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

Adafruit_VL53L1X vlx3 = Adafruit_VL53L1X();
Adafruit_VL53L1X vlx4 = Adafruit_VL53L1X();

bool sensor3Ok = false;
bool sensor4Ok = false;

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

    Serial.println("=== Drive + VLX (canales 3 y 4) ===");

    tcaSelect(3);
    if (!vlx3.begin(0x29, &Wire1))
    {
        Serial.println("ERROR inicializando VL53L1X canal 3");
    }
    else
    {
        sensor3Ok = vlx3.startRanging();
        Serial.println(sensor3Ok ? "VL53L1X canal 3 OK" : "ERROR iniciando ranging canal 3");
    }

    tcaSelect(4);
    if (!vlx4.begin(0x29, &Wire1))
    {
        Serial.println("ERROR inicializando VL53L1X canal 4");
    }
    else
    {
        sensor4Ok = vlx4.startRanging();
        Serial.println(sensor4Ok ? "VL53L1X canal 4 OK" : "ERROR iniciando ranging canal 4");
    }

    drive.begin();
    drive.holdYaw(true);
    drive.setTargetYaw(drive.getYaw());
}

void loop()
{
    uint32_t roundStart = millis();

    // =========================
    // CANAL 3 -- igual que vlx_single_test.cpp
    // =========================
    tcaSelect(3);

    Serial.print("Canal 3: ");

    int16_t distancia3 = -1;
    if (sensor3Ok && vlx3.dataReady())
    {
        distancia3 = vlx3.distance();

        if (distancia3 == -1)
        {
            Serial.print("ERROR");
        }
        else
        {
            Serial.print(distancia3);
            Serial.print(" mm");
        }

        vlx3.clearInterrupt();
    }
    else
    {
        Serial.print(sensor3Ok ? "esperando..." : "sensor no inicializado");
    }

    Serial.print("   |   ");

    // =========================
    // CANAL 4 -- igual que vlx_single_test.cpp
    // =========================
    tcaSelect(4);

    Serial.print("Canal 4: ");

    int16_t distancia4 = -1;
    if (sensor4Ok && vlx4.dataReady())
    {
        distancia4 = vlx4.distance();

        if (distancia4 == -1)
        {
            Serial.print("ERROR");
        }
        else
        {
            Serial.print(distancia4);
            Serial.print(" mm");
        }

        vlx4.clearInterrupt();
    }
    else
    {
        Serial.print(sensor4Ok ? "esperando..." : "sensor no inicializado");
    }

    // =========================
    // Decision de movimiento
    // =========================
    bool detect4 = (distancia4 != -1) && (distancia4 < kDetectMm);

    if (detect4)
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
