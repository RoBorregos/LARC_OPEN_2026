/**
 * @file rtos_bno085_only.cpp
 * @brief RTOS (FreeRTOS) reading ONLY the BNO085, no ToF/mux/IR. Isolates
 *        whether the sensor responds inside the scheduler before wiring it
 *        up with the rest of the sensors.
 *
 * Target: Teensy 4.1 (Arduino framework, PlatformIO)
 *
 * Tasks:
 *   - taskReadBNO       (HIGH priority) 20 Hz -> reads yaw/pitch/roll
 *   - taskHealthMonitor (LOW priority)  1 Hz  -> prints reading + accuracy + free stack
 *
 * Uses Adafruit_BNO08x (SH-2 protocol) instead of hand-rolled registers like
 * the BNO055 version, since SH-2 framing isn't practical to reimplement.
 * getSensorEvent() doesn't block waiting for a report, but Wire can still
 * hang on a stretched bus, so access stays guarded by mutex + timeout.
 *
 * Uses the vendored lib/FreeRTOS-Teensy4 (MSP fix in prvPortStartFirstTask(),
 * see docs/2026-08-12-rtos-i2c-debug.md). vTaskStartScheduler() is called at
 * the end of setup(); loop() never runs again once the scheduler starts.
 *
 * pio run -e rtos_bno085_only -t upload -t monitor
 */

#include <Arduino.h>
#include <Wire.h>

#include <FreeRTOS_TEENSY4.h>

#include <Adafruit_BNO08x.h>

// ============================================================
// BNO085 configuration
// ============================================================

constexpr uint8_t BNO_ADDRESS_PRIMARY   = 0x4A; // ADR to GND
constexpr uint8_t BNO_ADDRESS_SECONDARY = 0x4B; // ADR to VCC
constexpr uint32_t I2C_FREQUENCY        = 100000; // antes 50000 (heredado del BNO055) -- a 50kHz begin_I2C()
                                                    // nunca completaba el handshake SH-2 (ver bno085_standalone.cpp)
constexpr uint8_t MAX_ATTEMPTS          = 20;

// El chip a veces deja de mandar reportes (getSensorEvent()/readBNO085()
// siempre false) sin que el bus se caiga -- lastYaw/Pitch/Roll se quedan
// pegados en el ultimo valor para siempre porque nada vuelve a pisarlos.
// Si pasa este tiempo sin una lectura valida, se fuerza un reinit completo
// (begin_I2C manda su propio software-reset SH-2 internamente).
constexpr uint32_t STALL_TIMEOUT_MS = 1500;

// Rotation report every 20 ms (50 Hz), faster than taskReadBNO's 20 Hz poll
// so a fresh reading is always waiting.
constexpr uint32_t REPORT_PERIOD_US = 20000;

static Adafruit_BNO08x bno; // no hardware reset pin (RST/INT nets unconfirmed on pins.h)

static uint8_t bnoAddress = BNO_ADDRESS_PRIMARY;
static bool    bnoReady   = false;

struct BnoReading
{
    float yaw    = 0.0f;
    float pitch  = 0.0f;
    float roll   = 0.0f;
    uint32_t timestamp = 0;
};

static volatile uint32_t bnoOkCount       = 0;
static volatile uint32_t bnoFailCount     = 0;
static volatile uint32_t bnoRecoveryCount = 0;

// Last decoded reading + accuracy (0=Unreliable .. 3=High), so the health
// monitor can print without touching I2C again.
static volatile float   lastYaw      = 0.0f;
static volatile float   lastPitch    = 0.0f;
static volatile float   lastRoll     = 0.0f;
static volatile uint8_t lastAccuracy = 0;

// ============================================================
// RTOS objects
// ============================================================

static QueueHandle_t     bnoQueue = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;

static TaskHandle_t hReadBNO       = nullptr;
static TaskHandle_t hHealthMonitor = nullptr;

// ============================================================
// BNO085 utilities
// ============================================================

static void quaternionToEuler(float qw, float qx, float qy, float qz,
                               float &yaw, float &pitch, float &roll)
{
    yaw = atan2f(2.0f * (qw * qz + qx * qy),
                 1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / PI);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp       = fmaxf(-1.0f, fminf(1.0f, sinp));
    pitch      = asinf(sinp) * (180.0f / PI);

    roll = atan2f(2.0f * (qw * qx + qy * qz),
                  1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / PI);
}

static bool tryBeginAt(uint8_t address)
{
    for (uint8_t attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt)
    {
        if (bno.begin_I2C(address, &Wire2))
        {
            bnoAddress = address;
            return true;
        }
        delay(100);
    }
    return false;
}

static bool initBNO085()
{
    if (!tryBeginAt(BNO_ADDRESS_PRIMARY))
    {
        Serial.println(F("[BNO085] No response at 0x4A, trying 0x4B..."));
        if (!tryBeginAt(BNO_ADDRESS_SECONDARY))
        {
            Serial.println(F("[BNO085] init FAILED: chip not detected at 0x4A or 0x4B"));
            return false;
        }
    }

    Serial.printf("[BNO085] begin_I2C OK at address 0x%02X\n", bnoAddress);

    if (!bno.enableReport(SH2_ROTATION_VECTOR, REPORT_PERIOD_US))
    {
        Serial.println(F("[BNO085] init FAILED: could not enable SH2_ROTATION_VECTOR"));
        return false;
    }

    Serial.println(F("[BNO085] init OK, rotation report enabled"));
    return true;
}

// value->status: 0=Unreliable 1=Low 2=Medium 3=High
static bool readBNO085(BnoReading &reading, uint8_t &accuracy)
{
    sh2_SensorValue_t value;
    if (!bno.getSensorEvent(&value))
        return false;

    if (value.sensorId != SH2_ROTATION_VECTOR)
        return false;

    const float qw = value.un.rotationVector.real;
    const float qx = value.un.rotationVector.i;
    const float qy = value.un.rotationVector.j;
    const float qz = value.un.rotationVector.k;

    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    quaternionToEuler(qw, qx, qy, qz, yaw, pitch, roll);

    if (yaw < 0.0f)
        yaw += 360.0f;

    reading.yaw   = yaw;
    reading.pitch = pitch;
    reading.roll  = roll;
    accuracy      = value.status & 0x03;

    return true;
}

// ============================================================
// RTOS tasks
// ============================================================

void taskReadBNO(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake   = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50); // 20 Hz
    TickType_t lastOkTick = lastWake;

    for (;;)
    {
        if (bnoReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            BnoReading reading;
            reading.timestamp = millis();

            uint8_t accuracy = 0;
            bool ok = readBNO085(reading, accuracy);

            if (ok)
            {
                ++bnoOkCount;
                lastYaw      = reading.yaw;
                lastPitch    = reading.pitch;
                lastRoll     = reading.roll;
                lastAccuracy = accuracy;
                lastOkTick   = xTaskGetTickCount();
                xQueueSend(bnoQueue, &reading, 0);
            }
            else
            {
                ++bnoFailCount;

                // Sin esto, lastYaw/Pitch/Roll quedan pegados para siempre si
                // el SH-2 deja de mandar reportes. begin_I2C() manda su propio
                // software-reset SH-2 (i2chal_open), no hace falta un pin de
                // reset de hardware para recuperarlo.
                if ((xTaskGetTickCount() - lastOkTick) > pdMS_TO_TICKS(STALL_TIMEOUT_MS))
                {
                    ++bnoRecoveryCount;
                    bnoReady   = initBNO085();
                    lastOkTick = xTaskGetTickCount();
                }
            }

            xSemaphoreGive(i2cMutex);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskHealthMonitor(void *pvParameters)
{
    (void)pvParameters;

    static const char *kAccuracyName[4] = {"Unreliable", "Low", "Medium", "High"};

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000);

    for (;;)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        Serial.printf("[t=%lu ms] ", (unsigned long)millis());
        Serial.print(F("Yaw="));
        Serial.print(lastYaw, 2);
        Serial.print(F(" Pitch="));
        Serial.print(lastPitch, 2);
        Serial.print(F(" Roll="));
        Serial.print(lastRoll, 2);
        Serial.print(F(" | "));

        Serial.printf(
            "accuracy=%s OK=%lu FAIL=%lu recoveries=%lu | stackFree ReadBNO=%u HealthMon=%u\n",
            kAccuracyName[lastAccuracy & 0x03],
            (unsigned long)bnoOkCount, (unsigned long)bnoFailCount,
            (unsigned long)bnoRecoveryCount,
            (unsigned)uxTaskGetStackHighWaterMark(hReadBNO),
            (unsigned)uxTaskGetStackHighWaterMark(hHealthMonitor));

        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================
// Arduino entry points
// ============================================================

// Blink code to diagnose where setup() hangs without needing the serial
// monitor open at boot time. N short blinks + long pause, repeated 3 times.
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
    Serial.println(F("[BOOT] setup() started"));
    blinkCode(1);

    // Teensy 4.1 (Wire2): SDA2 = pin 25, SCL2 = pin 24
    Wire2.begin();
    Wire2.setClock(I2C_FREQUENCY);
    Serial.println(F("[BOOT] Wire2 ready, starting BNO085..."));
    blinkCode(2);

    bnoReady = initBNO085();
    Serial.printf("[BOOT] BNO085 bnoReady=%d\n", bnoReady);
    blinkCode(3);

    bnoQueue = xQueueCreate(5, sizeof(BnoReading));
    i2cMutex = xSemaphoreCreateMutex();
    Serial.println(F("[BOOT] queue/mutex created, creating tasks..."));
    blinkCode(4);

    xTaskCreate(taskReadBNO,       "ReadBNO",   2048, nullptr, 2, &hReadBNO);
    xTaskCreate(taskHealthMonitor, "HealthMon", 1024, nullptr, 1, &hHealthMonitor);
    blinkCode(5);
    digitalWrite(LED_BUILTIN, HIGH); // solid = about to call vTaskStartScheduler()

    Serial.println(F("[RTOS] Starting scheduler..."));
    vTaskStartScheduler();

    // Only reached if there wasn't enough RAM to start.
    Serial.println(F("[RTOS] vTaskStartScheduler() failed (insufficient RAM)"));
    while (true) {}
}

void loop()
{
    // Unused: control never returns here once vTaskStartScheduler() runs.
}
