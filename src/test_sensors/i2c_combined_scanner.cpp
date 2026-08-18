/**
 * @file i2c_combined_scanner.cpp
 * @brief Escaneo combinado y continuo del bus I2C: en cada ronda, hace
 *        ping directo al BNO055 (bus principal, fuera del mux) Y
 *        escanea los 8 canales del TCA9548A buscando los ToF (VLX),
 *        todo en el mismo loop -- para ver el estado de ambos "al mismo
 *        tiempo" en vez de subir un firmware distinto para cada uno.
 *
 *        Incluye recuperacion de bus I2C (bit-bang manual de SCL) antes
 *        de cada ronda: en el escaneo anterior (ver src/tca_scanner.cpp)
 *        el TCA9548A se colgaba a partir de cierto canal y dejaba de
 *        responder incluso en canales que antes funcionaban. Si el
 *        select de un canal falla, este sketch intenta destrabar el bus
 *        y reintenta una vez antes de reportar error.
 *
 * pio run -e i2c_combined_scanner -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_PIN = 18;
constexpr uint8_t SCL_PIN = 19;

constexpr uint8_t BNO_ADDRESS_PRIMARY   = 0x28;
constexpr uint8_t BNO_ADDRESS_SECONDARY = 0x29;
constexpr uint8_t REG_CHIP_ID           = 0x00;
constexpr uint8_t EXPECTED_BNO_CHIP_ID  = 0xA0;

constexpr uint8_t  TCA_ADDRESS    = 0x70;
constexpr uint8_t  NUM_CHANNELS   = 8;

constexpr uint32_t I2C_FREQUENCY  = 50000;
constexpr uint32_t SCAN_PERIOD_MS = 3000;

// ============================================================
// Recuperacion de bus I2C
// ============================================================
// Si un esclavo (o una linea flotante sin pull-up) deja SDA atascado en
// LOW a mitad de una transaccion, el bus queda trabado y NADA responde
// despues, incluido el propio mux. Esto libera manualmente el bus
// generando hasta 9 pulsos de reloj en SCL (como recomienda la nota de
// aplicacion de NXP para I2C bus recovery) y una condicion STOP.
void recoverI2CBus()
{
    Wire.end();

    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);

    for (uint8_t i = 0; i < 9; i++)
    {
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(5);

        if (digitalRead(SDA_PIN) == HIGH)
        {
            break;
        }
    }

    // Genera STOP manual: SDA de LOW a HIGH mientras SCL esta en HIGH.
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(5);

    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
}

// ============================================================
// TCA9548A
// ============================================================

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

bool selectChannelRaw(uint8_t channel)
{
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(static_cast<uint8_t>(1 << channel));
    return Wire.endTransmission() == 0;
}

bool selectChannelWithRecovery(uint8_t channel)
{
    if (selectChannelRaw(channel))
    {
        return true;
    }

    Serial.println("    [recuperando bus I2C...]");
    recoverI2CBus();

    return selectChannelRaw(channel);
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

const char* guessToF(uint8_t address)
{
    if (address == 0x29)
    {
        return "VL53L0X/VL53L1X (default)";
    }
    return "";
}

void scanChannel(uint8_t channel)
{
    Serial.print("  Canal ");
    Serial.print(channel);
    Serial.print(" (");
    Serial.print(channelLabel(channel));
    Serial.println("):");

    if (!selectChannelWithRecovery(channel))
    {
        Serial.println("    ERROR: TCA9548A no respondio (incluso tras recuperar bus).");
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
            Serial.print("    ");
            printAddress(address);
            Serial.print("  ACK");

            const char* guess = guessToF(address);
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
        Serial.println("    (nada respondio)");
    }
}

// ============================================================
// BNO055 -- ping directo, bus principal (no pasa por el mux)
// ============================================================

bool pingBNO(uint8_t& foundAddress, uint8_t& chipId)
{
    const uint8_t candidates[2] = {BNO_ADDRESS_PRIMARY, BNO_ADDRESS_SECONDARY};

    for (uint8_t candidate : candidates)
    {
        Wire.beginTransmission(candidate);
        Wire.write(REG_CHIP_ID);

        if (Wire.endTransmission(false) != 0)
        {
            continue;
        }

        if (Wire.requestFrom(candidate, static_cast<uint8_t>(1), static_cast<uint8_t>(true)) != 1)
        {
            continue;
        }

        chipId = Wire.read();
        foundAddress = candidate;
        return true;
    }

    return false;
}

// ============================================================
// Arduino
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("  I2C COMBINED SCANNER (BNO055 + ToF/mux)");
    Serial.println("=========================================");

    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("I2C initialized");
}

void loop()
{
    static uint32_t previousScanTime = 0;
    static uint32_t roundNumber = 0;

    if (millis() - previousScanTime < SCAN_PERIOD_MS)
    {
        return;
    }

    previousScanTime = millis();
    roundNumber++;

    Serial.println();
    Serial.print("=== Ronda ");
    Serial.print(roundNumber);
    Serial.print(" | t=");
    Serial.print(previousScanTime);
    Serial.println("ms ===");

    uint8_t bnoAddress = 0;
    uint8_t bnoChipId  = 0;

    if (pingBNO(bnoAddress, bnoChipId))
    {
        Serial.print("[BNO055] OK  @0x");
        printAddress(bnoAddress);
        Serial.print("  CHIP_ID=0x");
        Serial.print(bnoChipId, HEX);
        Serial.print(bnoChipId == EXPECTED_BNO_CHIP_ID ? "  (esperado)" : "  (INESPERADO)");
        Serial.println();
    }
    else
    {
        Serial.println("[BNO055] NO RESPONDIO (0x28 ni 0x29)");
    }

    Serial.println("[ToF / TCA9548A] escaneando canales...");
    for (uint8_t channel = 0; channel < NUM_CHANNELS; channel++)
    {
        scanChannel(channel);
    }
    disableAllChannels();
}
