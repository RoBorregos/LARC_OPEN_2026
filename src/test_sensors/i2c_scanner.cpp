/**
 * @file i2c_scanner.cpp
 * @brief Escaneo del bus I2C completo (sin RTOS, sin asumir ningun
 *        dispositivo de antemano): recorre las 127 direcciones de 7 bits
 *        y reporta cuales responden. Sirve para confirmar en vivo, en el
 *        cableado actual, si el BNO055 (0x28/0x29), el TCA9548A
 *        (0x70-0x77) o cualquier otro modulo esta realmente vivo en el
 *        bus antes de asumir que un driver especifico esta mal.
 *
 * pio run -e i2c_scanner -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

constexpr uint32_t I2C_FREQUENCY  = 50000;
constexpr uint32_t SCAN_PERIOD_MS = 3000;

const char* guessDevice(uint8_t address)
{
    switch (address)
    {
        case 0x28:
        case 0x29:
            return "BNO055";

        case 0x70:
        case 0x71:
        case 0x72:
        case 0x73:
        case 0x74:
        case 0x75:
        case 0x76:
        case 0x77:
            return "TCA9548A (mux)";

        default:
            if (address >= 0x40 && address <= 0x7F)
            {
                return "posible PCA9685";
            }
            return "";
    }
}

void printAddress(uint8_t address)
{
    Serial.print("0x");
    if (address < 0x10)
    {
        Serial.print("0");
    }
    Serial.print(address, HEX);
}

void scanBus()
{
    Serial.println("Escaneando bus I2C (0x01-0x7F)...");

    uint8_t found = 0;

    for (uint8_t address = 0x01; address <= 0x7F; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            found++;

            Serial.print("  ");
            printAddress(address);
            Serial.print("  ACK");

            const char* guess = guessDevice(address);
            if (guess[0] != '\0')
            {
                Serial.print("  <- ");
                Serial.print(guess);
            }

            Serial.println();
        }
        else if (error != 2)
        {
            // error 2 = NACK en la direccion (nada conectado ahi, normal).
            // Cualquier otro codigo es una anomalia de bus que vale la
            // pena ver, aunque no haya dispositivo confirmado.
            Serial.print("  ");
            printAddress(address);
            Serial.print("  error ");
            Serial.println(error);
        }
    }

    if (found == 0)
    {
        Serial.println("Ningun dispositivo respondio (ACK). Revisar alimentacion, pull-ups y continuidad de SDA/SCL.");
    }
    else
    {
        Serial.print(found);
        Serial.println(" dispositivo(s) encontrados.");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("        I2C SCANNER");
    Serial.println("================================");

    // Teensy 4.1:
    // SDA = pin 18
    // SCL = pin 19
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("I2C initialized");
}

void loop()
{
    static uint32_t previousScanTime = 0;

    if (millis() - previousScanTime < SCAN_PERIOD_MS)
    {
        return;
    }

    previousScanTime = millis();

    Serial.println();
    scanBus();
}
