// pio run -e rtos_statemachine_test -t upload -t monitor

#include <Arduino.h>

#include <FreeRTOS_TEENSY4.h>

#include "PeriodicRunner.hpp"
#include "robot/StateMachine/AntiqueStateMachine/StateMachine.hpp"
#include "robot/instances/instances.hpp"

LARCStateMachine stateMachine;

namespace
{
    PeriodicRunner robotSchedule(20);
    PeriodicRunner actuatorSchedule(20);
}

static TaskHandle_t hControlLoop = nullptr;
static TaskHandle_t hOdometry    = nullptr;
static TaskHandle_t hSensors     = nullptr;
static TaskHandle_t hHealthMonitor = nullptr;

static SemaphoreHandle_t i2cMutex = nullptr;

void taskControlLoop(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20);

    for (;;)
    {
        const uint32_t now = millis();

        if (robotSchedule.due(now))
        {
            stateMachine.update();
        }

        if (actuatorSchedule.due(now))
        {
            servos.update();
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskOdometry(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    for (;;)
    {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            stateMachine.updateControl();
            xSemaphoreGive(i2cMutex);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskSensors(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(30);

    for (;;)
    {
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            tofLeft.update();
            tofRight.update();
            xSemaphoreGive(i2cMutex);
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

void taskHealthMonitor(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2000);

    for (;;)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        Serial.printf(
            "[HEALTH %lu ms] Stack free (words) ControlLoop=%u Odometry=%u Sensors=%u HealthMon=%u | "
            "ToF valid(L,R)=%d,%d dist(L,R)=%.1f,%.1f cm\n",
            (unsigned long)millis(),
            (unsigned)uxTaskGetStackHighWaterMark(hControlLoop),
            (unsigned)uxTaskGetStackHighWaterMark(hOdometry),
            (unsigned)uxTaskGetStackHighWaterMark(hSensors),
            (unsigned)uxTaskGetStackHighWaterMark(hHealthMonitor),
            (int)tofLeft.isValid(), (int)tofRight.isValid(),
            tofLeft.getDistanceCm(), tofRight.getDistanceCm());

        vTaskDelayUntil(&lastWake, period);
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println(F("[BOOT] setup() started"));

    ir.begin();
    qtrFront.begin();
    servos.begin();
    elevator.begin();

    stateMachine.begin();
    Serial.println(F("[BOOT] stateMachine.begin() done, creating mutex/tasks..."));

    i2cMutex = xSemaphoreCreateMutex();

    xTaskCreate(taskOdometry,      "Odometry",    2048, nullptr, 3, &hOdometry);
    xTaskCreate(taskControlLoop,   "ControlLoop", 4096, nullptr, 2, &hControlLoop);
    xTaskCreate(taskSensors,       "Sensors",     1024, nullptr, 2, &hSensors);
    xTaskCreate(taskHealthMonitor, "HealthMon",   1024, nullptr, 1, &hHealthMonitor);

    Serial.println(F("[RTOS] Starting scheduler..."));
    vTaskStartScheduler();

    Serial.println(F("[RTOS] vTaskStartScheduler() failed (insufficient RAM)"));
    while (true) {}
}

void loop()
{
}
