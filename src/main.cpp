#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================
// Configuración del BNO055
// ============================================================

constexpr uint8_t BNO_ADDRESS = 0x28;

// Teensy 4.1:
// SDA -> pin 18
// SCL -> pin 19

constexpr uint32_t I2C_FREQUENCY = 50000;
constexpr uint32_t SAMPLE_PERIOD_MS = 500;
constexpr uint8_t MAX_ATTEMPTS = 5;

// ============================================================
// Configuración del PCA9685
// ============================================================

constexpr uint8_t TCA_ADDRESS = 0x70;

// Servo conectado al canal 8 del PCA9685
constexpr uint8_t SERVO_CHANNEL = 8;

// Posiciones conservadoras para la prueba
constexpr uint16_t SERVO_LEFT   = 260;
constexpr uint16_t SERVO_CENTER = 320;
constexpr uint16_t SERVO_RIGHT  = 380;

constexpr uint32_t SERVO_MOVEMENT_PERIOD_MS = 1500;

Adafruit_PWMServoDriver* pca = nullptr;

// ============================================================
// Registros BNO055
// ============================================================

constexpr uint8_t REG_CHIP_ID        = 0x00;
constexpr uint8_t REG_PAGE_ID        = 0x07;
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
        Wire.beginTransmission(BNO_ADDRESS);
        Wire.write(registerAddress);
        Wire.write(value);

        uint8_t error = Wire.endTransmission(true);

        if (error == 0)
        {
            delayMicroseconds(100);
            return true;
        }

        Serial.print("Fallo escritura I2C. Registro 0x");
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

        Wire.beginTransmission(BNO_ADDRESS);
        Wire.write(startRegister);

        // Repeated START
        uint8_t error = Wire.endTransmission(false);

        if (error != 0)
        {
            Serial.print("endTransmission fallo. Intento ");
            Serial.print(attempt);
            Serial.print(" | Error: ");
            Serial.println(error);

            delay(3);
            continue;
        }

        uint8_t received = Wire.requestFrom(
            BNO_ADDRESS,
            length,
            static_cast<uint8_t>(true)
        );

        if (received != length)
        {
            Serial.print("requestFrom incompleto. Intento ");
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
                Serial.println("Buffer Wire incompleto.");
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
// Inicialización del BNO055
// ============================================================

bool initializeBNO055()
{
    Serial.println("Buscando BNO055...");

    uint8_t chipId = 0;

    for (uint8_t attempt = 1; attempt <= 20; attempt++)
    {
        if (readRegister(REG_CHIP_ID, chipId))
        {
            Serial.print("CHIP_ID: 0x");
            Serial.println(chipId, HEX);

            if (chipId == EXPECTED_CHIP_ID)
            {
                break;
            }
        }

        delay(100);
    }

    if (chipId != EXPECTED_CHIP_ID)
    {
        Serial.println("ERROR: BNO055 no reconocido.");
        return false;
    }

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

    Serial.print("Modo actual: 0x");
    Serial.println(operationMode, HEX);

    if (operationMode != MODE_NDOF)
    {
        Serial.println("ERROR: El sensor no entro en NDOF.");
        return false;
    }

    Serial.println("BNO055 inicializado correctamente.");
    return true;
}

// ============================================================
// Lectura exclusiva del yaw
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
        Serial.print("Yaw fuera de rango: ");
        Serial.println(yaw);
        return false;
    }

    return true;
}

// ============================================================
// Funciones del PCA9685
// ============================================================

uint8_t findPCAAddress()
{
    Serial.println("Buscando PCA9685 en el bus principal...");

    for (uint8_t address = 0x40; address <= 0x7F; address++)
    {
        // Se conserva 0x70 para el TCA9548A.
        if (address == TCA_ADDRESS)
        {
            continue;
        }

        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Posible PCA9685 encontrado en 0x");

            if (address < 0x10)
            {
                Serial.print("0");
            }

            Serial.println(address, HEX);
            return address;
        }
    }

    return 0;
}

void moveServo(uint16_t pulse)
{
    if (pca != nullptr)
    {
        pca->setPWM(SERVO_CHANNEL, 0, pulse);
    }
}

bool initializePCA9685()
{
    uint8_t pcaAddress = findPCAAddress();

    if (pcaAddress == 0)
    {
        Serial.println();
        Serial.println("ERROR: No se encontro el PCA9685.");
        Serial.println("El dispositivo 0x70 es el TCA9548A.");
        Serial.println("Revisa VCC, GND, SDA, SCL, OE y A0-A5.");
        return false;
    }

    pca = new Adafruit_PWMServoDriver(pcaAddress, Wire);

    Serial.print("Inicializando PCA9685 en 0x");
    Serial.println(pcaAddress, HEX);

    pca->begin();
    pca->setPWMFreq(50);

    delay(500);

    Serial.println("Colocando servo del canal 8 al centro...");
    moveServo(SERVO_CENTER);

    delay(1000);

    return true;
}

// ============================================================
// Movimiento no bloqueante del servo
// ============================================================

void updateServo()
{
    static uint32_t previousMovementTime = 0;
    static uint8_t movementState = 0;

    if (millis() - previousMovementTime < SERVO_MOVEMENT_PERIOD_MS)
    {
        return;
    }

    previousMovementTime = millis();

    switch (movementState)
    {
        case 0:
            Serial.println("Servo canal 8 -> IZQUIERDA");
            moveServo(SERVO_LEFT);
            movementState = 1;
            break;

        case 1:
            Serial.println("Servo canal 8 -> DERECHA");
            moveServo(SERVO_RIGHT);
            movementState = 2;
            break;

        default:
            Serial.println("Servo canal 8 -> CENTRO");
            moveServo(SERVO_CENTER);
            movementState = 0;
            break;
    }
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
    Serial.println("   PRUEBA BNO055 + SERVO PCA");
    Serial.println("================================");

    // Teensy 4.1:
    // SDA = pin 18
    // SCL = pin 19
    Wire.begin();

    // Se conserva la frecuencia utilizada por el código
    // estable del BNO055.
    Wire.setClock(I2C_FREQUENCY);

    if (!initializeBNO055())
    {
        Serial.println("No se pudo inicializar el BNO055.");

        while (true)
        {
            delay(1000);
        }
    }

    if (!initializePCA9685())
    {
        Serial.println("No se pudo inicializar el PCA9685.");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("Comenzando prueba...");
}

void loop()
{
    static uint32_t previousYawTime = 0;
    static uint32_t validReadings = 0;
    static uint32_t failedReadings = 0;

    // Mueve el servo sin detener las lecturas del BNO055.
    updateServo();

    // Lee el yaw cada 500 ms.
    if (millis() - previousYawTime >= SAMPLE_PERIOD_MS)
    {
        previousYawTime = millis();

        float yaw = 0.0f;

        if (readYaw(yaw))
        {
            validReadings++;

            Serial.print("Yaw: ");
            Serial.print(yaw, 2);
            Serial.print(" grados");

            Serial.print(" | Correctas: ");
            Serial.print(validReadings);

            Serial.print(" | Fallidas: ");
            Serial.println(failedReadings);
        }
        else
        {
            failedReadings++;

            Serial.print("ERROR: No se pudo leer yaw.");

            Serial.print(" | Correctas: ");
            Serial.print(validReadings);

            Serial.print(" | Fallidas: ");
            Serial.println(failedReadings);
        }
    }
}