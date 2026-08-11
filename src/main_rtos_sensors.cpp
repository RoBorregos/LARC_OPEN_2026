/**
 * @file main_rtos_sensors.cpp
 * @brief Prueba de RTOS real (FreeRTOS) con lectura de sensores, previa a la
 *        integración con LARCStateMachine.
 *
 * Equipo: RoBorregos - LARC Open
 * Target: Teensy 4.1 (Arduino framework, PlatformIO)
 *
 * Este archivo se compila con el entorno dedicado `rtos_sensor_test` (ver
 * platformio.ini, build_src_filter) y queda excluido del entorno `teensy41`
 * normal, para no interferir con el main.cpp real de la máquina de estados
 * mientras se valida el RTOS.
 *
 * Tareas:
 *   - taskReadSensors   (prioridad ALTA)  50 Hz  -> lee BNO055, ToF, IR
 *   - taskProcessData   (prioridad MEDIA)        -> consume la queue, imprime cada 500 ms
 *   - taskHealthMonitor (prioridad BAJA)  cada 5 s -> stack libre + calibración BNO055
 *
 * NOTA: usa la librería discord-intech/FreeRTOS-Teensy4 (ver lib_deps del
 * entorno rtos_sensor_test). Su header maestro es FreeRTOS_TEENSY4.h, que
 * ya incluye FreeRTOS.h/task.h/queue.h/semphr.h/portmacro.h internamente.
 * A diferencia de otros ports "estilo AVR", este NO arranca el scheduler
 * solo: hay que llamar vTaskStartScheduler() explícitamente al final de
 * setup() (ver abajo). Mientras el scheduler corre, loop() nunca se
 * vuelve a ejecutar.
 */

#include <Arduino.h>
#include <Wire.h>

#include <FreeRTOS_TEENSY4.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L0X.h>
#include "TCA9548A/TCA9548A.h"

#include "pins.h"

// =============================================================
// Datos compartidos entre tareas
// =============================================================

static constexpr uint8_t kTofCount     = 4;
static constexpr uint16_t kTofInvalidMm = 0xFFFF; // sin lectura válida / sensor no conectado

struct SensorData
{
    float    yaw   = 0.0f;
    float    pitch = 0.0f;
    float    roll  = 0.0f;
    uint16_t tof_distances[kTofCount] = {kTofInvalidMm, kTofInvalidMm, kTofInvalidMm, kTofInvalidMm};
    int      ir_value = 0;
    uint32_t timestamp = 0;
};

// =============================================================
// Instancias de sensores
// =============================================================

// BNO055 crudo (no el wrapper de lib/Sensors/BNO) porque esta prueba
// necesita acceso directo a getCalibration() para el health monitor.
static Adafruit_BNO055 bno(55, 0x28, &Wire);
static bool bnoReady = false;

// ToF: mismo mux/canales que usa el resto del proyecto (ver pins.h),
// pero AÚN NO están conectados físicamente -> begin() puede fallar por
// sensor y eso es esperado. La estructura queda lista para cuando se
// conecten.
static TCA9548A i2cMux(0x70);
static VL53L0X  tofSensors[kTofCount];
static bool     tofInitialized[kTofCount] = {false, false, false, false};
static const uint8_t tofChannels[kTofCount] = {
    Pins::kToFchFR,
    Pins::kToFchFL,
    Pins::kToFchBL,
    Pins::kToFchBR
};

// IR: lectura analógica simple e independiente del arreglo IRLine
// (que vive detrás del mux 74HC4067). Placeholder hasta definir el
// pin real en pins.h.
static constexpr uint8_t kIrTestPin = A0; // TODO: reemplazar por el pin real del IR analógico

// =============================================================
// Objetos RTOS
// =============================================================

static QueueHandle_t     sensorQueue   = nullptr;
static SemaphoreHandle_t i2cMutex      = nullptr;

static TaskHandle_t hReadSensors   = nullptr;
static TaskHandle_t hProcessData   = nullptr;
static TaskHandle_t hHealthMonitor = nullptr;

// =============================================================
// Helpers de inicialización
// =============================================================

static bool initBNO055()
{
    if (!bno.begin())
    {
        Serial.println(F("[BNO055] init FALLO - revisar cableado / direccion 0x28"));
        return false;
    }

    delay(50);
    bno.setExtCrystalUse(true);
    Serial.println(F("[BNO055] init OK"));
    return true;
}

static bool initTOFSensors()
{
    if (!i2cMux.begin())
    {
        Serial.println(F("[TCA9548A] mux no responde en 0x70 - ToF omitidos"));
        return false;
    }

    bool anyOk = false;
    for (uint8_t i = 0; i < kTofCount; ++i)
    {
        i2cMux.selectChannel(tofChannels[i]);
        tofSensors[i].setTimeout(200);

        tofInitialized[i] = tofSensors[i].init();
        if (tofInitialized[i])
        {
            tofSensors[i].startContinuous();
            anyOk = true;
            Serial.printf("[ToF ch%u] init OK\n", tofChannels[i]);
        }
        else
        {
            Serial.printf("[ToF ch%u] no conectado (esperado por ahora)\n", tofChannels[i]);
        }
    }

    return anyOk;
}

// =============================================================
// Helpers de lectura (asumen que el caller ya tomó i2cMutex cuando aplica)
// =============================================================

static void readBNO055(SensorData &data)
{
    if (!bnoReady)
        return;

    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    data.yaw   = event.orientation.x;
    data.pitch = event.orientation.y;
    data.roll  = event.orientation.z;
}

static void readTOFSensors(SensorData &data)
{
    for (uint8_t i = 0; i < kTofCount; ++i)
    {
        if (!tofInitialized[i])
        {
            data.tof_distances[i] = kTofInvalidMm;
            continue;
        }

        i2cMux.selectChannel(tofChannels[i]);
        uint16_t mm = tofSensors[i].readRangeContinuousMillimeters();
        data.tof_distances[i] = tofSensors[i].timeoutOccurred() ? kTofInvalidMm : mm;
    }
}

static int readIRSensor()
{
    // Puramente analógico, no comparte el bus I2C -> no necesita el mutex.
    return analogRead(kIrTestPin);
}

// =============================================================
// Tareas RTOS
// =============================================================

void taskReadSensors(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    for (;;)
    {
        SensorData data;
        data.timestamp = millis();

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            readBNO055(data);
            readTOFSensors(data);
            xSemaphoreGive(i2cMutex);
        }

        data.ir_value = readIRSensor();

        // No bloqueante: si el consumidor va atrasado, se descarta el dato
        // más viejo en vez de frenar la tarea de mayor prioridad.
        xQueueSend(sensorQueue, &data, 0);

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskProcessData(void *pvParameters)
{
    (void)pvParameters;

    SensorData data;
    uint32_t lastPrintMs = 0;

    for (;;)
    {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE)
        {
            const uint32_t now = millis();
            if (now - lastPrintMs >= 500)
            {
                lastPrintMs = now;
                Serial.printf(
                    "[%lu ms] Yaw=%.2f Pitch=%.2f Roll=%.2f | ToF[FR,FL,BL,BR]=%u,%u,%u,%u mm | IR=%d\n",
                    (unsigned long)data.timestamp,
                    data.yaw, data.pitch, data.roll,
                    data.tof_distances[0], data.tof_distances[1],
                    data.tof_distances[2], data.tof_distances[3],
                    data.ir_value);
            }
        }
    }
}

void taskHealthMonitor(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5000);

    for (;;)
    {
        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;

        if (bnoReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            bno.getCalibration(&sys, &gyro, &accel, &mag);
            xSemaphoreGive(i2cMutex);
        }

        Serial.printf(
            "[HEALTH %lu ms] BNO055 cal(sys=%u gyro=%u accel=%u mag=%u) | "
            "Stack libre (words) ReadSensors=%u ProcessData=%u HealthMon=%u\n",
            (unsigned long)millis(), sys, gyro, accel, mag,
            (unsigned)uxTaskGetStackHighWaterMark(hReadSensors),
            (unsigned)uxTaskGetStackHighWaterMark(hProcessData),
            (unsigned)uxTaskGetStackHighWaterMark(hHealthMonitor));

        vTaskDelayUntil(&lastWake, period);
    }
}

// =============================================================
// Arduino entry points
// =============================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Wire.begin();
    Wire.setClock(400000);

    bnoReady = initBNO055();
    initTOFSensors();

    sensorQueue = xQueueCreate(5, sizeof(SensorData));
    i2cMutex    = xSemaphoreCreateMutex();

    xTaskCreate(taskReadSensors,   "ReadSensors", 2048, nullptr, 3, &hReadSensors);
    xTaskCreate(taskProcessData,   "ProcessData", 2048, nullptr, 2, &hProcessData);
    xTaskCreate(taskHealthMonitor, "HealthMon",   1024, nullptr, 1, &hHealthMonitor);

    Serial.println(F("[RTOS] Arrancando scheduler..."));
    vTaskStartScheduler();

    // Solo se llega aquí si no hubo RAM suficiente para arrancar.
    Serial.println(F("[RTOS] vTaskStartScheduler() fallo (RAM insuficiente)"));
    while (true) {}
}

void loop()
{
    // No se usa: una vez que vTaskStartScheduler() corre, el control
    // nunca regresa aquí.
}
