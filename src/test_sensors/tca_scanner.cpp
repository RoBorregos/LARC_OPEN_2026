/**
 * @file tca_scanner.cpp
 * @brief Escaneo canal por canal del TCA9548A (mux I2C, confirmado vivo en
 *        0x70 por env:i2c_scanner): para cada uno de los 8 canales, lo
 *        selecciona y escanea el bus I2C completo, reportando que
 *        responde detras de cada uno. Sirve para confirmar cuales ToF
 *        (VL53L0X/VL53L1X, tipicamente 0x29 por default) estan realmente
 *        conectados en los canales FR/FL/BL/BR (ver pins.h: kToFchFR=0,
 *        kToFchFL=1, kToFchBL=2, kToFchBR=3).
 *
 * pio run -e tca_scanner -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t  TCA_ADDRESS     = 0x70;
constexpr uint32_t I2C_FREQUENCY   = 50000;
constexpr uint32_t SCAN_PERIOD_MS  = 4000;
constexpr uint8_t  NUM_CHANNELS    = 8;

const char* channelLabel(uint8_t channel)
{
    switch (channel)
    {
        case 0: return "FR (kToFchFR)";
        case 1: return "FL (kToFchFL)";
        case 2: return "BL (kToFchBL)";
        case 3: return "BR (kToFchBR)";
        default: return "sin rol asignado en pins.h";
    }
}

const char* guessDevice(uint8_t address)
{
    switch (address)
    {
        case 0x29:
            return "VL53L0X/VL53L1X (default)";
        case 0x28:
            return "BNO055 (raro detras del mux)";
        default:
            return "";
    }
}

bool selectChannel(uint8_t channel)
{
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(static_cast<uint8_t>(1 << channel));
    return Wire.endTransmission() == 0;
}

void disableAllChannels()
{
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();
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

void scanChannel(uint8_t channel)
{
    Serial.print("--- Canal ");
    Serial.print(channel);
    Serial.print(" (");
    Serial.print(channelLabel(channel));
    Serial.println(") ---");

    if (!selectChannel(channel))
    {
        Serial.println("  ERROR: no se pudo seleccionar el canal (TCA9548A no respondio).");
        return;
    }

    uint8_t found = 0;

    for (uint8_t address = 0x01; address <= 0x7F; address++)
    {
        if (address == TCA_ADDRESS)
        {
            continue;
        }

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
    }

    if (found == 0)
    {
        Serial.println("  (nada respondio en este canal)");
    }
}

void scanAllChannels()
{
    Serial.println();
    Serial.println("Escaneando los 8 canales del TCA9548A...");

    for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++)
    {
        scanChannel(channel);
    }

    disableAllChannels();
    Serial.println("--- fin de ronda ---");
}

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   TCA9548A CHANNEL SCANNER");
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
    scanAllChannels();
}
