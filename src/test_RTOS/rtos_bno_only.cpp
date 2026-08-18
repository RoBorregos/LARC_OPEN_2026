/**
 * @file rtos_bno_only.cpp
 * @brief RTOS (FreeRTOS) leyendo UNICAMENTE el BNO055, sin ToF, sin mux
 *        TCA9548A y sin IR. Aisla si el BNO055 responde corriendo dentro
 *        del scheduler una vez que ya se confirmo con env:i2c_scanner que
 *        direccion esta viva en el bus.
 *
 * Target: Teensy 4.1 (Arduino framework, PlatformIO)
 *
 * Tareas:
 *   - taskReadBNO       (prioridad ALTA) 20 Hz    -> lee yaw/pitch/roll
 *   - taskHealthMonitor (prioridad BAJA) cada 1 s -> imprime lectura + calibracion + stack libre
 *
 * NOTA: usa la libreria vendorizada en lib/FreeRTOS-Teensy4 (con el fix
 * del MSP en prvPortStartFirstTask(), ver docs/2026-08-12-rtos-i2c-debug.md).
 * Su header maestro es FreeRTOS_TEENSY4.h. Hay que llamar
 * vTaskStartScheduler() explicitamente al final de setup(); loop() nunca
 * se vuelve a ejecutar mientras el scheduler corre.
 *
 * pio run -e rtos_bno_only -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include <FreeRTOS_TEENSY4.h>

// ============================================================
// Configuración del BNO055
// ============================================================

constexpr uint8_t BNO_ADDRESS_PRIMARY   = 0x28;
constexpr uint8_t BNO_ADDRESS_SECONDARY = 0x29;
constexpr uint32_t I2C_FREQUENCY        = 50000;
constexpr uint8_t MAX_ATTEMPTS          = 5;

static uint8_t bnoAddress = BNO_ADDRESS_PRIMARY;
static bool    bnoReady   = false;

constexpr uint8_t REG_CHIP_ID        = 0x00;
constexpr uint8_t REG_PAGE_ID        = 0x07;
constexpr uint8_t REG_CALIB_STAT     = 0x35;
constexpr uint8_t REG_EULER_HEADING  = 0x1A; // heading, roll, pitch (6 bytes)
constexpr uint8_t REG_OPERATION_MODE = 0x3D;
constexpr uint8_t REG_POWER_MODE     = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER    = 0x3F;

constexpr uint8_t EXPECTED_CHIP_ID = 0xA0;
constexpr uint8_t MODE_CONFIG      = 0x00;
constexpr uint8_t MODE_NDOF        = 0x0C;
constexpr uint8_t POWER_NORMAL     = 0x00;

struct BnoReading
{
    float yaw    = 0.0f;
    float pitch  = 0.0f;
    float roll   = 0.0f;
    uint32_t timestamp = 0;
};

// Contadores de diagnostico, mismo criterio que main_rtos_sensors.cpp:
// distinguen "el sensor da 0.00 real" de "la lectura esta fallando".
static volatile uint32_t bnoOkCount   = 0;
static volatile uint32_t bnoFailCount = 0;

// Diagnostico: ultimos 6 bytes crudos leidos de REG_EULER_HEADING (0x1A),
// para distinguir "el chip devuelve ceros" de "hay un bug al pedir/parsear".
static volatile uint8_t lastRawEuler[6] = {0, 0, 0, 0, 0, 0};

// Ultima lectura decodificada, para que el health monitor imprima el dato
// mas reciente en vez de sacarlo de bnoQueue (que se satura a 20 Hz contra
// 1 consumo/seg y termina mostrando lecturas viejas).
static volatile float lastYaw   = 0.0f;
static volatile float lastPitch = 0.0f;
static volatile float lastRoll  = 0.0f;

// ============================================================
// Objetos RTOS
// ============================================================

static QueueHandle_t     bnoQueue = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;

static TaskHandle_t hReadBNO       = nullptr;
static TaskHandle_t hHealthMonitor = nullptr;

// ============================================================
// I2C de bajo nivel del BNO055 (mismo driver que bno_servo_standalone.cpp)
// ============================================================

static bool bnoWriteRegister(uint8_t registerAddress, uint8_t value)
{
    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt)
    {
        Wire.beginTransmission(bnoAddress);
        Wire.write(registerAddress);
        Wire.write(value);

        if (Wire.endTransmission(true) == 0)
        {
            delayMicroseconds(100);
            return true;
        }

        delay(3);
    }

    return false;
}

static bool bnoReadRegisters(uint8_t startRegister, uint8_t *buffer, uint8_t length)
{
    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt)
    {
        while (Wire.available())
            Wire.read();

        Wire.beginTransmission(bnoAddress);
        Wire.write(startRegister);

        if (Wire.endTransmission(false) != 0) // repeated START
        {
            delay(3);
            continue;
        }

        uint8_t received = Wire.requestFrom(bnoAddress, length, static_cast<uint8_t>(true));
        if (received != length)
        {
            while (Wire.available())
                Wire.read();
            delay(3);
            continue;
        }

        for (uint8_t i = 0; i < length; ++i)
        {
            if (!Wire.available())
                return false;
            buffer[i] = Wire.read();
        }

        return true;
    }

    return false;
}

static bool bnoReadRegister(uint8_t registerAddress, uint8_t &value)
{
    return bnoReadRegisters(registerAddress, &value, 1);
}

static bool probeChipId(uint8_t address, uint8_t &chipId)
{
    bnoAddress = address;
    return bnoReadRegister(REG_CHIP_ID, chipId) && chipId == EXPECTED_CHIP_ID;
}

static bool initBNO055()
{
    uint8_t chipId = 0;
    bool found = false;

    for (uint8_t attempt = 1; attempt <= 20 && !found; ++attempt)
    {
        found = probeChipId(BNO_ADDRESS_PRIMARY, chipId);
        if (!found)
            delay(100);
    }

    if (!found)
    {
        Serial.println(F("[BNO055] No respondio en 0x28, probando 0x29..."));

        for (uint8_t attempt = 1; attempt <= 20 && !found; ++attempt)
        {
            found = probeChipId(BNO_ADDRESS_SECONDARY, chipId);
            if (!found)
                delay(100);
        }
    }

    if (!found)
    {
        Serial.println(F("[BNO055] init FALLO: chip no detectado en 0x28 ni 0x29"));
        return false;
    }

    Serial.printf("[BNO055] CHIP_ID=0x%02X en direccion 0x%02X\n", chipId, bnoAddress);

    if (!bnoWriteRegister(REG_OPERATION_MODE, MODE_CONFIG))
        return false;
    delay(25);

    if (!bnoWriteRegister(REG_PAGE_ID, 0x00))
        return false;

    if (!bnoWriteRegister(REG_POWER_MODE, POWER_NORMAL))
        return false;
    delay(10);

    if (!bnoWriteRegister(REG_SYS_TRIGGER, 0x00)) // oscilador interno
        return false;
    delay(10);

    if (!bnoWriteRegister(REG_OPERATION_MODE, MODE_NDOF))
        return false;
    delay(30);

    uint8_t operationMode = 0;
    if (!bnoReadRegister(REG_OPERATION_MODE, operationMode))
        return false;

    if ((operationMode & 0x0F) != MODE_NDOF)
    {
        Serial.println(F("[BNO055] init FALLO: no entro en modo NDOF"));
        return false;
    }

    Serial.println(F("[BNO055] init OK, modo NDOF activo"));
    return true;
}

static bool readBNO055(BnoReading &reading)
{
    // heading(2) + roll(2) + pitch(2), LSB primero, 1 grado = 16 LSB
    uint8_t buffer[6];
    if (!bnoReadRegisters(REG_EULER_HEADING, buffer, sizeof(buffer)))
        return false;

    for (uint8_t i = 0; i < sizeof(buffer); ++i)
        lastRawEuler[i] = buffer[i];

    auto toAngle = [](uint8_t lsb, uint8_t msb) -> float {
        int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(lsb) | (static_cast<uint16_t>(msb) << 8));
        return static_cast<float>(raw) / 16.0f;
    };

    const float yaw = toAngle(buffer[0], buffer[1]);
    if (yaw < 0.0f || yaw >= 360.0f)
        return false;

    reading.yaw   = yaw;
    reading.roll  = toAngle(buffer[2], buffer[3]);
    reading.pitch = toAngle(buffer[4], buffer[5]);
    return true;
}

static void readBNO055Calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t &mag)
{
    uint8_t calib = 0;
    if (!bnoReadRegister(REG_CALIB_STAT, calib))
    {
        sys = gyro = accel = mag = 0;
        return;
    }

    sys   = (calib >> 6) & 0x03;
    gyro  = (calib >> 4) & 0x03;
    accel = (calib >> 2) & 0x03;
    mag   = calib & 0x03;
}

// ============================================================
// Tareas RTOS
// ============================================================

void taskReadBNO(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz

    for (;;)
    {
        if (bnoReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            BnoReading reading;
            reading.timestamp = millis();

            bool ok = readBNO055(reading);
            xSemaphoreGive(i2cMutex);

            if (ok)
            {
                ++bnoOkCount;
                lastYaw   = reading.yaw;
                lastPitch = reading.pitch;
                lastRoll  = reading.roll;
                xQueueSend(bnoQueue, &reading, 0);
            }
            else
            {
                ++bnoFailCount;
            }
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskHealthMonitor(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    for (;;)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;

        if (bnoReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            readBNO055Calibration(sys, gyro, accel, mag);
            xSemaphoreGive(i2cMutex);
        }

        Serial.printf("[t=%lu ms] ", (unsigned long)millis());
        Serial.print(F("Yaw="));
        Serial.print(lastYaw, 2);
        Serial.print(F(" Pitch="));
        Serial.print(lastPitch, 2);
        Serial.print(F(" Roll="));
        Serial.print(lastRoll, 2);
        Serial.print(F(" | "));

        Serial.printf(
            "cal(sys=%u gyro=%u accel=%u mag=%u) OK=%lu FAIL=%lu | stackFree ReadBNO=%u HealthMon=%u | "
            "raw0x1A=%02X %02X %02X %02X %02X %02X\n",
            sys, gyro, accel, mag,
            (unsigned long)bnoOkCount, (unsigned long)bnoFailCount,
            (unsigned)uxTaskGetStackHighWaterMark(hReadBNO),
            (unsigned)uxTaskGetStackHighWaterMark(hHealthMonitor),
            lastRawEuler[0], lastRawEuler[1], lastRawEuler[2],
            lastRawEuler[3], lastRawEuler[4], lastRawEuler[5]);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================
// Arduino entry points
// ============================================================

// Codigo de parpadeos para diagnosticar en donde se cuelga setup() sin
// depender de que alguien tenga el monitor serie abierto en el momento
// exacto del boot. N parpadeos cortos + pausa larga; se repite 3 veces.
static void blinkCode(uint8_t n)
{
    for (uint8_t rep = 0; rep < 3; ++rep)
    {
        for (uint8_t i = 0; i < n; ++i)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(150);
            digitalWrite(LED_BUILTIN, LOW);
            delay(150);
        }
        delay(800);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println(F("[BOOT] setup() arranco"));
    blinkCode(1);

    // Teensy 4.1:
    // SDA = pin 18
    // SCL = pin 19
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY);
    Serial.println(F("[BOOT] Wire listo, iniciando BNO055..."));
    blinkCode(2);

    bnoReady = initBNO055();
    Serial.printf("[BOOT] BNO055 bnoReady=%d\n", bnoReady);
    blinkCode(3);

    bnoQueue = xQueueCreate(5, sizeof(BnoReading));
    i2cMutex = xSemaphoreCreateMutex();
    Serial.println(F("[BOOT] queue/mutex creados, creando tareas..."));
    blinkCode(4);

    xTaskCreate(taskReadBNO,       "ReadBNO",   2048, nullptr, 2, &hReadBNO);
    xTaskCreate(taskHealthMonitor, "HealthMon", 1024, nullptr, 1, &hHealthMonitor);
    blinkCode(5);
    digitalWrite(LED_BUILTIN, HIGH); // solido = a punto de llamar vTaskStartScheduler()

    Serial.println(F("[RTOS] Arrancando scheduler..."));
    vTaskStartScheduler();

    // Solo se llega aqui si no hubo RAM suficiente para arrancar.
    Serial.println(F("[RTOS] vTaskStartScheduler() fallo (RAM insuficiente)"));
    while (true) {}
}

void loop()
{
    // No se usa: una vez que vTaskStartScheduler() corre, el control
    // nunca regresa aqui.
}
