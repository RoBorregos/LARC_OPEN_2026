/**
 * @file vlx_single_test.cpp
 * @brief Diagnostico standalone (sin motores/BNO/IR/RTOS) de dos ToF (VLX,
 *        VL53L1X) conectados detras del TCA9548A, en el bus I2C1 (Wire1,
 *        SDA1=17/SCL1=16), canales 3 y 4 del mux.
 *
 *        La seleccion de canal usa Wire1 directo (tcaSelect), sin pasar
 *        por la clase TCA9548A -- confirmado funcionando en hardware real
 *        con este patron. Usa Adafruit_VL53L1X (no la clase ToF de
 *        lib/sensors/tof, que usa la libreria Pololu y no expone forma
 *        de elegir el bus I2C) -- tambien confirmado funcionando.
 *
 * pio run -e vlx_single_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>

constexpr uint8_t kTcaAddress = 0x70;

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

    Serial.println("=== TCA9548A + Adafruit VL53L1X (canales 3 y 4) ===");

    // =========================
    // VL53L1X CANAL 3
    // =========================
    tcaSelect(3);

    if (!vlx3.begin(0x29, &Wire1))
    {
        Serial.println("ERROR inicializando VL53L1X canal 3");
    }
    else
    {
        Serial.println("VL53L1X canal 3 OK");
        sensor3Ok = vlx3.startRanging();
        if (!sensor3Ok)
        {
            Serial.println("ERROR iniciando ranging canal 3");
        }
    }

    // =========================
    // VL53L1X CANAL 4
    // =========================
    tcaSelect(4);

    if (!vlx4.begin(0x29, &Wire1))
    {
        Serial.println("ERROR inicializando VL53L1X canal 4");
    }
    else
    {
        Serial.println("VL53L1X canal 4 OK");
        sensor4Ok = vlx4.startRanging();
        if (!sensor4Ok)
        {
            Serial.println("ERROR iniciando ranging canal 4");
        }
    }

    Serial.println();
    Serial.println("Iniciando lecturas...");
}

void loop()
{
    // =========================
    // CANAL 3
    // =========================
    tcaSelect(3);

    Serial.print("Canal 3: ");

    if (sensor3Ok && vlx3.dataReady())
    {
        int16_t distancia3 = vlx3.distance();

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
    // CANAL 4
    // =========================
    tcaSelect(4);

    Serial.print("Canal 4: ");

    if (sensor4Ok && vlx4.dataReady())
    {
        int16_t distancia4 = vlx4.distance();

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

    Serial.println();

    delay(100);
}
