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

// BNO055 por registros crudos (sin Adafruit_BNO055): la librería quedó
// descartada porque sus llamadas bloqueantes de I2C pueden colgarse sin
// límite y, al correr dentro de taskReadSensors (prioridad ALTA), eso
// acaparaba el scheduler para siempre y ni taskHealthMonitor llegaba a
// imprimir. Este driver usa reintentos acotados (MAX_ATTEMPTS) y el reloj
// I2C a 50 kHz, que es la configuración ya validada como estable para este
// BNO055 (ver prueba independiente BNO055+PCA9685).
static constexpr uint8_t  kBnoAddress   = 0x28;
static constexpr uint32_t kI2cFrequency = 50000;
static constexpr uint8_t  kMaxI2cAttempts = 5;

static constexpr uint8_t kRegChipId       = 0x00;
static constexpr uint8_t kRegPageId       = 0x07;
static constexpr uint8_t kRegEulerHeading = 0x1A; // heading, roll, pitch (6 bytes)
static constexpr uint8_t kRegCalibStat    = 0x35;
static constexpr uint8_t kRegOperationMode = 0x3D;
static constexpr uint8_t kRegPowerMode     = 0x3E;
static constexpr uint8_t kRegSysTrigger    = 0x3F;

static constexpr uint8_t kExpectedChipId = 0xA0;
static constexpr uint8_t kModeConfig  = 0x00;
static constexpr uint8_t kModeNdof    = 0x0C;
static constexpr uint8_t kPowerNormal = 0x00;

static bool bnoReady = false;

// Contadores de diagnostico: cuantas lecturas del Euler del BNO055
// realmente tienen exito vs fallan (bnoReadRegisters devuelve false o el
// yaw sale fuera de rango). Sirven para distinguir "el sensor da 0.00
// real" de "la lectura esta fallando en silencio y el struct se queda en
// su valor default". Solo los incrementa taskReadSensors; los lee
// taskHealthMonitor para imprimirlos (no hace falta mutex extra, son
// contadores de diagnostico, no estado critico).
static volatile uint32_t bnoEulerOkCount   = 0;
static volatile uint32_t bnoEulerFailCount = 0;

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

static bool bnoWriteRegister(uint8_t registerAddress, uint8_t value)
{
    for (uint8_t attempt = 1; attempt <= kMaxI2cAttempts; ++attempt)
    {
        Wire.beginTransmission(kBnoAddress);
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
    for (uint8_t attempt = 1; attempt <= kMaxI2cAttempts; ++attempt)
    {
        while (Wire.available())
            Wire.read();

        Wire.beginTransmission(kBnoAddress);
        Wire.write(startRegister);

        if (Wire.endTransmission(false) != 0) // repeated START
        {
            delay(3);
            continue;
        }

        uint8_t received = Wire.requestFrom(kBnoAddress, length, static_cast<uint8_t>(true));
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

static bool initBNO055()
{
    uint8_t chipId = 0;

    for (uint8_t attempt = 1; attempt <= 20; ++attempt)
    {
        if (bnoReadRegister(kRegChipId, chipId) && chipId == kExpectedChipId)
            break;
        delay(100);
    }

    if (chipId != kExpectedChipId)
    {
        Serial.println(F("[BNO055] init FALLO - chip no detectado en 0x28"));
        return false;
    }

    if (!bnoWriteRegister(kRegOperationMode, kModeConfig))
        return false;
    delay(25);

    if (!bnoWriteRegister(kRegPageId, 0x00))
        return false;

    if (!bnoWriteRegister(kRegPowerMode, kPowerNormal))
        return false;
    delay(10);

    if (!bnoWriteRegister(kRegSysTrigger, 0x00)) // oscilador interno
        return false;
    delay(10);

    if (!bnoWriteRegister(kRegOperationMode, kModeNdof))
        return false;
    delay(30);

    uint8_t operationMode = 0;
    if (!bnoReadRegister(kRegOperationMode, operationMode))
        return false;

    if ((operationMode & 0x0F) != kModeNdof)
    {
        Serial.println(F("[BNO055] init FALLO - no entro en modo NDOF"));
        return false;
    }

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

// Devuelve true solo si se pudieron leer los 6 bytes Y el yaw cae en el
// rango valido [0,360) del heading del BNO055 (mismo criterio que la
// prueba standalone BNO055+servo que ya funcionaba). Un false aqui es lo
// que hace que data.yaw/pitch/roll se queden en su default de 0.0f.
static bool readBNO055(SensorData &data)
{
    if (!bnoReady)
        return false;

    // heading(2) + roll(2) + pitch(2), LSB primero, 1 grado = 16 LSB
    uint8_t buffer[6];
    if (!bnoReadRegisters(kRegEulerHeading, buffer, sizeof(buffer)))
        return false;

    auto toAngle = [](uint8_t lsb, uint8_t msb) -> float {
        int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(lsb) | (static_cast<uint16_t>(msb) << 8));
        return static_cast<float>(raw) / 16.0f;
    };

    const float yaw = toAngle(buffer[0], buffer[1]);
    if (yaw < 0.0f || yaw >= 360.0f)
        return false;

    data.yaw   = yaw;
    data.roll  = toAngle(buffer[2], buffer[3]);
    data.pitch = toAngle(buffer[4], buffer[5]);
    return true;
}

static void readBNO055Calibration(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t &mag)
{
    uint8_t calib = 0;
    if (!bnoReadRegister(kRegCalibStat, calib))
    {
        sys = gyro = accel = mag = 0;
        return;
    }

    sys   = (calib >> 6) & 0x03;
    gyro  = (calib >> 4) & 0x03;
    accel = (calib >> 2) & 0x03;
    mag   = calib & 0x03;
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
            if (readBNO055(data))
                ++bnoEulerOkCount;
            else
                ++bnoEulerFailCount;
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
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;

        if (bnoReady && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            readBNO055Calibration(sys, gyro, accel, mag);
            xSemaphoreGive(i2cMutex);
        }

        Serial.printf(
            "[HEALTH %lu ms] BNO055 cal(sys=%u gyro=%u accel=%u mag=%u) Euler(ok=%lu fail=%lu) | "
            "Stack libre (words) ReadSensors=%u ProcessData=%u HealthMon=%u\n",
            (unsigned long)millis(), sys, gyro, accel, mag,
            (unsigned long)bnoEulerOkCount, (unsigned long)bnoEulerFailCount,
            (unsigned)uxTaskGetStackHighWaterMark(hReadSensors),
            (unsigned)uxTaskGetStackHighWaterMark(hProcessData),
            (unsigned)uxTaskGetStackHighWaterMark(hHealthMonitor));

        vTaskDelayUntil(&lastWake, period);
    }
}

// =============================================================
// Arduino entry points
// =============================================================

// Codigo de parpadeos para diagnosticar en donde se cuelga setup() sin
// depender de que alguien tenga el monitor serie abierto en el momento
// exacto del boot (los prints de setup() se pierden si nadie escucha).
// N parpadeos cortos + pausa larga; se repite 3 veces por checkpoint.
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

    Wire.begin();
    Wire.setClock(kI2cFrequency); // 50 kHz: validado como estable para el BNO055
    Serial.println(F("[BOOT] Wire listo, iniciando BNO055..."));
    blinkCode(2);

    bnoReady = initBNO055();
    Serial.printf("[BOOT] BNO055 bnoReady=%d, iniciando ToF...\n", bnoReady);
    blinkCode(3);

    initTOFSensors();
    Serial.println(F("[BOOT] ToF listo, creando objetos RTOS..."));
    blinkCode(4);

    sensorQueue = xQueueCreate(5, sizeof(SensorData));
    i2cMutex    = xSemaphoreCreateMutex();
    Serial.println(F("[BOOT] queue/mutex creados, creando tareas..."));
    blinkCode(5);

    xTaskCreate(taskReadSensors,   "ReadSensors", 2048, nullptr, 3, &hReadSensors);
    xTaskCreate(taskProcessData,   "ProcessData", 2048, nullptr, 2, &hProcessData);
    xTaskCreate(taskHealthMonitor, "HealthMon",   1024, nullptr, 1, &hHealthMonitor);
    blinkCode(6);
    digitalWrite(LED_BUILTIN, HIGH); // solido = a punto de llamar vTaskStartScheduler()

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
