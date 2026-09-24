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

// rangeStatus del VL53L1X (ULD de ST). Adafruit distance() regresa -1 para
// cualquier status != 0, asi que lo leemos directo para saber la causa.
const char* rangeStatusText(uint8_t status)
{
    switch (status)
    {
        case 0:  return "OK";
        case 1:  return "sigma fail (ruido)";
        case 2:  return "signal fail (poca luz / nada enfrente)";
        case 4:  return "fuera de rango";
        case 5:  return "hardware fail";
        case 7:  return "wraparound (muy lejos)";
        default: return "desconocido";
    }
}

void printReading(Adafruit_VL53L1X& vlx, bool ok)
{
    if (!ok)
    {
        Serial.print("sensor no inicializado");
        return;
    }
    if (!vlx.dataReady())
    {
        Serial.print("esperando...");
        return;
    }

    uint8_t status = 0;
    uint16_t distancia = 0;
    bool i2cOk = vlx.VL53L1X_GetRangeStatus(&status) == 0 &&
                 vlx.VL53L1X_GetDistance(&distancia) == 0;

    if (!i2cOk)
    {
        Serial.print("ERROR I2C");
    }
    else if (status == 0)
    {
        Serial.print(distancia);
        Serial.print(" mm");
    }
    else
    {
        Serial.print("INVALIDO st=");
        Serial.print(status);
        Serial.print(" ");
        Serial.print(rangeStatusText(status));
        Serial.print(" (raw ");
        Serial.print(distancia);
        Serial.print(" mm)");
    }

    vlx.clearInterrupt();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Wire1.begin();
    Wire1.setClock(100000);

    Serial.println("=== TCA9548A + Adafruit VL53L1X (canales 3 y 4) ===");

    // =========================
    // VL53L1X CANALES
    // =========================
    // Que VLX corresponde a que canal y ubicacion en el chassis:
    // Canal 0 es para el de UR
    // Canal 2 es para UL
    // Canal 3 es para LL
    // Canal 4 es para LR

    tcaSelect(0); // 3 VL53 

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
    tcaSelect(2);

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
    tcaSelect(0);

    Serial.print("Canal 3 (TCA 0): ");
    printReading(vlx3, sensor3Ok);

    Serial.print("   |   ");

    // =========================
    // CANAL 4
    // =========================
    tcaSelect(2);

    Serial.print("Canal 4 (TCA 2): ");
    printReading(vlx4, sensor4Ok);

    Serial.println();

    delay(100);
}
