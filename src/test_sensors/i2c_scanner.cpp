/**
 * @file i2c_scanner.cpp
 * @brief Escaneo del bus I2C completo (sin RTOS, sin asumir ningun
 *        dispositivo de antemano): recorre las 127 direcciones de 7 bits
 *        y reporta cuales responden. Sirve para confirmar en vivo, en el
 *        cableado actual, si el BNO055 (0x28/0x29), el BNO085 (0x4A/0x4B),
 *        el TCA9548A (0x70-0x77) o cualquier otro modulo esta realmente
 *        vivo en el bus antes de asumir que un driver especifico esta mal.
 *
 *        Modo "live": escanea cada SCAN_PERIOD_MS y solo imprime cuando
 *        una direccion aparece o desaparece (con timestamp). Pensado para
 *        mover/presionar cables mientras corre y ver en tiempo real el
 *        instante exacto en que un dispositivo se cae del bus.
 *
 * pio run -e i2c_scanner -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

constexpr uint32_t I2C_FREQUENCY  = 50000;
constexpr uint32_t SCAN_PERIOD_MS = 100;

const char* guessDevice(uint8_t address)
{
    switch (address)
    {
        case 0x28:
        case 0x29:
            return "BNO055";

        case 0x4A:
        case 0x4B:
            return "BNO085";

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

static bool present[128] = {false};
static bool baselineDone = false;
static uint32_t dropCount[128] = {0};

void printTimestamp()
{
    Serial.print("[t=");
    Serial.print(millis());
    Serial.print("ms] ");
}

// Escanea una sola vez y reporta unicamente los CAMBIOS de estado respecto
// a la vuelta anterior (una direccion que aparece o desaparece), no el
// estado completo cada vez. Sirve para ver en vivo, mientras se mueven
// cables, el instante exacto en que un dispositivo se cae o vuelve.
void scanBusChanges()
{
    for (uint8_t address = 0x01; address <= 0x7F; address++)
    {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        bool ack = (error == 0);

        if (ack == present[address])
        {
            continue;
        }

        present[address] = ack;

        if (!baselineDone)
        {
            // Primera pasada: no es una "caida", es el estado inicial.
            continue;
        }

        printTimestamp();
        printAddress(address);

        if (ack)
        {
            Serial.print("  APARECIO (ACK)");
        }
        else
        {
            dropCount[address]++;
            Serial.print("  SE CAYO (NACK/error ");
            Serial.print(error);
            Serial.print(") -- caidas acumuladas: ");
            Serial.print(dropCount[address]);
        }

        const char* guess = guessDevice(address);
        if (guess[0] != '\0')
        {
            Serial.print("  <- ");
            Serial.print(guess);
        }

        Serial.println();
    }

    if (!baselineDone)
    {
        baselineDone = true;

        Serial.println("Estado inicial:");
        uint8_t found = 0;
        for (uint16_t address = 0x01; address <= 0x7F; address++)
        {
            if (present[address])
            {
                found++;
                Serial.print("  ");
                printAddress((uint8_t)address);
                Serial.print("  ACK");

                const char* guess = guessDevice((uint8_t)address);
                if (guess[0] != '\0')
                {
                    Serial.print("  <- ");
                    Serial.print(guess);
                }
                Serial.println();
            }
        }
        if (found == 0)
        {
            Serial.println("  ninguno");
        }
        Serial.println("Escaneando en vivo... mueve/presiona los cables y observa los cambios.");
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

    scanBusChanges();
}
