/**
 * @file bno_servo_standalone.cpp
 * @brief Diagnostico puro del BNO055, sin RTOS y sin TCA9548A: si el sensor
 *        responde aqui pero no en el resto del sistema, el problema esta en
 *        el mux o su rama del PCB. Si sigue sin responder, el problema es
 *        mas general del bus I2C (alimentacion, pull-ups, continuidad).
 *
 * pio run -e bno_servo_standalone -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

// ============================================================
// Configuración del BNO055
// ============================================================

constexpr uint8_t BNO_ADDRESS_PRIMARY   = 0x28;
constexpr uint8_t BNO_ADDRESS_SECONDARY = 0x29;

// Teensy 4.1:
// SDA -> pin 18
// SCL -> pin 19

constexpr uint32_t I2C_FREQUENCY    = 50000;
constexpr uint32_t SAMPLE_PERIOD_MS = 1000;
constexpr uint8_t MAX_ATTEMPTS      = 5;

// Direccion detectada en initializeBNO055(); usada por el resto de
// funciones I2C.
uint8_t bnoAddress = BNO_ADDRESS_PRIMARY;

// ============================================================
// Registros BNO055
// ============================================================

constexpr uint8_t REG_CHIP_ID        = 0x00;
constexpr uint8_t REG_PAGE_ID        = 0x07;
constexpr uint8_t REG_CALIB_STAT     = 0x35;
constexpr uint8_t REG_SYS_STATUS     = 0x39;
constexpr uint8_t REG_EULER_HEADING  = 0x1A;
constexpr uint8_t REG_OPERATION_MODE = 0x3D;
constexpr uint8_t REG_POWER_MODE     = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER    = 0x3F;

constexpr uint8_t EXPECTED_CHIP_ID = 0xA0;

constexpr uint8_t MODE_CONFIG  = 0x00;
constexpr uint8_t MODE_NDOF    = 0x0C;
constexpr uint8_t POWER_NORMAL = 0x00;

// ============================================================
// Funciones I2C del BNO055
// ============================================================

bool writeRegister(uint8_t registerAddress, uint8_t value)
{
    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)
    {
        Wire.beginTransmission(bnoAddress);
        Wire.write(registerAddress);
        Wire.write(value);

        uint8_t error = Wire.endTransmission(true);

        if (error == 0)
        {
            delayMicroseconds(100);
            return true;
        }

        Serial.print("[I2C] Fallo escritura. Registro 0x");
        Serial.print(registerAddress, HEX);
        Serial.print(" | Intento ");
        Serial.print(attempt);
        Serial.print(" | Error: ");
        Serial.println(error);

        delay(3);
    }

    return false;
}

bool readRegisters(
    uint8_t startRegister,
    uint8_t* buffer,
    uint8_t length
)
{
    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)
    {
        while (Wire.available())
        {
            Wire.read();
        }

        Wire.beginTransmission(bnoAddress);
        Wire.write(startRegister);

        // Repeated START
        uint8_t error = Wire.endTransmission(false);

        if (error != 0)
        {
            Serial.print("[I2C] endTransmission fallo. Intento ");
            Serial.print(attempt);
            Serial.print(" | Error: ");
            Serial.println(error);

            delay(3);
            continue;
        }

        uint8_t received = Wire.requestFrom(
            bnoAddress,
            length,
            static_cast<uint8_t>(true)
        );

        if (received != length)
        {
            Serial.print("[I2C] requestFrom incompleto. Intento ");
            Serial.print(attempt);
            Serial.print(" | Esperados: ");
            Serial.print(length);
            Serial.print(" | Recibidos: ");
            Serial.println(received);

            while (Wire.available())
            {
                Wire.read();
            }

            delay(3);
            continue;
        }

        for (uint8_t index = 0; index < length; index++)
        {
            if (!Wire.available())
            {
                Serial.println("[I2C] Buffer Wire incompleto.");
                return false;
            }

            buffer[index] = Wire.read();
        }

        return true;
    }

    return false;
}

bool readRegister(uint8_t registerAddress, uint8_t& value)
{
    return readRegisters(registerAddress, &value, 1);
}

// ============================================================
// Detección de dirección + inicialización del BNO055
// ============================================================

bool probeChipId(uint8_t address, uint8_t& chipId)
{
    bnoAddress = address;
    return readRegister(REG_CHIP_ID, chipId) && chipId == EXPECTED_CHIP_ID;
}

bool initializeBNO055()
{
    Serial.println("[BNO055] Buscando en 0x28...");

    uint8_t chipId = 0;
    bool found = false;

    for (uint8_t attempt = 1; attempt <= 20 && !found; attempt++)
    {
        found = probeChipId(BNO_ADDRESS_PRIMARY, chipId);

        if (!found)
        {
            delay(100);
        }
    }

    if (!found)
    {
        Serial.println("[BNO055] No respondió en 0x28, probando 0x29...");

        for (uint8_t attempt = 1; attempt <= 20 && !found; attempt++)
        {
            found = probeChipId(BNO_ADDRESS_SECONDARY, chipId);

            if (!found)
            {
                delay(100);
            }
        }
    }

    if (!found)
    {
        Serial.println("[BNO055] init FALLO: chip no detectado en 0x28 ni 0x29.");
        return false;
    }

    Serial.print("[BNO055] init OK: CHIP_ID=0x");
    Serial.print(chipId, HEX);
    Serial.print(" en dirección 0x");
    Serial.println(bnoAddress, HEX);

    // Entrar en modo configuración
    if (!writeRegister(REG_OPERATION_MODE, MODE_CONFIG))
    {
        return false;
    }

    delay(25);

    // Seleccionar página 0
    if (!writeRegister(REG_PAGE_ID, 0x00))
    {
        return false;
    }

    // Modo de energía normal
    if (!writeRegister(REG_POWER_MODE, POWER_NORMAL))
    {
        return false;
    }

    delay(10);

    // Oscilador interno
    if (!writeRegister(REG_SYS_TRIGGER, 0x00))
    {
        return false;
    }

    delay(10);

    // Activar fusión de sensores
    if (!writeRegister(REG_OPERATION_MODE, MODE_NDOF))
    {
        return false;
    }

    delay(30);

    uint8_t operationMode = 0;

    if (!readRegister(REG_OPERATION_MODE, operationMode))
    {
        return false;
    }

    operationMode &= 0x0F;

    if (operationMode != MODE_NDOF)
    {
        Serial.print("[BNO055] ERROR: no entró en NDOF, modo actual=0x");
        Serial.println(operationMode, HEX);
        return false;
    }

    Serial.println("[BNO055] Modo NDOF activo. Inicialización completa.");
    return true;
}

// ============================================================
// Lecturas de diagnóstico
// ============================================================

bool readYaw(float& yaw)
{
    uint8_t yawBuffer[2];

    if (!readRegisters(
            REG_EULER_HEADING,
            yawBuffer,
            sizeof(yawBuffer)))
    {
        return false;
    }

    uint16_t unsignedRaw =
        static_cast<uint16_t>(yawBuffer[0]) |
        (static_cast<uint16_t>(yawBuffer[1]) << 8);

    int16_t rawYaw = static_cast<int16_t>(unsignedRaw);

    yaw = static_cast<float>(rawYaw) / 16.0f;

    if (yaw < 0.0f || yaw >= 360.0f)
    {
        Serial.print("[BNO055] Yaw fuera de rango: ");
        Serial.println(yaw);
        return false;
    }

    return true;
}

bool readSystemStatus(uint8_t& sysStatus, uint8_t& calibStatus)
{
    return readRegister(REG_SYS_STATUS, sysStatus) &&
           readRegister(REG_CALIB_STAT, calibStatus);
}

// ============================================================
// Arduino
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("   DIAGNOSTICO BNO055 (sin mux)");
    Serial.println("================================");

    // Teensy 4.1:
    // SDA = pin 18
    // SCL = pin 19
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    Serial.println("I2C initialized");

    if (!initializeBNO055())
    {
        Serial.println("[BNO055] No se pudo inicializar. Deteniendo.");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("Comenzando lecturas...");
}

void loop()
{
    static uint32_t previousReadTime = 0;
    static uint32_t validReadings = 0;
    static uint32_t failedReadings = 0;

    if (millis() - previousReadTime < SAMPLE_PERIOD_MS)
    {
        return;
    }

    previousReadTime = millis();

    float yaw = 0.0f;
    uint8_t sysStatus = 0;
    uint8_t calibStatus = 0;

    bool yawOk = readYaw(yaw);
    bool statusOk = readSystemStatus(sysStatus, calibStatus);

    Serial.print("[t=");
    Serial.print(previousReadTime);
    Serial.print("ms] ");

    if (yawOk)
    {
        validReadings++;
        Serial.print("Yaw: ");
        Serial.print(yaw, 2);
        Serial.print(" grados");
    }
    else
    {
        failedReadings++;
        Serial.print("Yaw: ERROR");
    }

    if (statusOk)
    {
        Serial.print(" | SYS_STATUS: 0x");
        Serial.print(sysStatus, HEX);
        Serial.print(" | CALIB(sys,gyro,accel,mag): ");
        Serial.print((calibStatus >> 6) & 0x03);
        Serial.print(",");
        Serial.print((calibStatus >> 4) & 0x03);
        Serial.print(",");
        Serial.print((calibStatus >> 2) & 0x03);
        Serial.print(",");
        Serial.print(calibStatus & 0x03);
    }
    else
    {
        Serial.print(" | SYS_STATUS: ERROR");
    }

    Serial.print(" | OK: ");
    Serial.print(validReadings);
    Serial.print(" | FAIL: ");
    Serial.println(failedReadings);
}
