/**
 * @file rtos_scheduler_test.cpp
 * @brief Test minimo: solo confirma que vTaskStartScheduler() funciona en
 *        este hardware/config, sin tocar I2C ni sensores. Sirve para
 *        aislar si un cuelgue viene del port de FreeRTOS o del codigo de
 *        sensores en main_rtos_sensors.cpp.
 *
 * pio run -e rtos_scheduler_test -t upload -t monitor
 */

#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

static void taskA(void *pvParameters)
{
    (void)pvParameters;
    pinMode(LED_BUILTIN, OUTPUT);
    uint32_t counter = 0;
    for (;;)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.printf("[A] tick %lu\n", (unsigned long)counter++);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void taskB(void *pvParameters)
{
    (void)pvParameters;
    uint32_t counter = 0;
    for (;;)
    {
        Serial.printf("[B] tick %lu\n", (unsigned long)counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println(F("[TEST] setup() OK, creando tareas..."));

    xTaskCreate(taskA, "A", 512, nullptr, 2, nullptr);
    xTaskCreate(taskB, "B", 512, nullptr, 1, nullptr);

    Serial.println(F("[TEST] Arrancando scheduler..."));
    vTaskStartScheduler();

    Serial.println(F("[TEST] vTaskStartScheduler() fallo (RAM insuficiente)"));
    while (true) {}
}

void loop() {}
