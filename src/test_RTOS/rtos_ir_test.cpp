/**
 * @file rtos_ir_test.cpp
 * @brief Combina el RTOS ya validado (ver rtos_scheduler_test.cpp, fix en
 *        lib/FreeRTOS-Teensy4/src/port.c) con la lectura de los 4 IR
 *        digitales usando los pines del esquematico nuevo (ver ir_test.cpp).
 *        Los IR son digitalRead() directo -> sin I2C, sin mutex, sin mux,
 *        asi que no hay riesgo de bloquear el scheduler como pasaba con el
 *        BNO055.
 *
 *        L1 -> pin 15
 *        L2 -> pin 14
 *        L3 -> pin 23
 *        L4 -> pin 41
 *
 * pio run -e rtos_ir_test -t upload -t monitor
 */

#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

struct IrPin
{
    const char *label;
    uint8_t pin;
};

static const IrPin kIrPins[4] = {
    {"L1", 15},
    {"L2", 14},
    {"L3", 23},
    {"L4", 41},
};

static void taskReadIR(void *pvParameters)
{
    (void)pvParameters;

    uint32_t sampleCount = 0;
    uint32_t transitions[4] = {0, 0, 0, 0};
    int lastValue[4] = {-1, -1, -1, -1};

    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        // Muestreo rapido (cada tick, ~1 kHz) para detectar ruido de pines
        // flotantes, igual que ir_test.cpp.
        for (uint8_t rep = 0; rep < 5; ++rep)
        {
            for (uint8_t i = 0; i < 4; ++i)
            {
                int v = digitalRead(kIrPins[i].pin);
                if (lastValue[i] != -1 && v != lastValue[i])
                    ++transitions[i];
                lastValue[i] = v;
            }
            ++sampleCount;
        }

        static uint32_t printCounter = 0;
        if (++printCounter >= 40) // ~200 ms si el loop de arriba tarda ~5 ticks
        {
            printCounter = 0;
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

            Serial.printf(
                "[%lu ms] L1=%d(%lu) L2=%d(%lu) L3=%d(%lu) L4=%d(%lu)  (de %lu muestras)\n",
                (unsigned long)millis(),
                lastValue[0], (unsigned long)transitions[0],
                lastValue[1], (unsigned long)transitions[1],
                lastValue[2], (unsigned long)transitions[2],
                lastValue[3], (unsigned long)transitions[3],
                (unsigned long)sampleCount);

            for (uint8_t i = 0; i < 4; ++i)
                transitions[i] = 0;
            sampleCount = 0;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5));
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    for (const auto &ir : kIrPins)
        pinMode(ir.pin, INPUT);

    Serial.println(F("[RTOS+IR] pines:"));
    for (const auto &ir : kIrPins)
        Serial.printf("  %s -> pin %u\n", ir.label, ir.pin);

    xTaskCreate(taskReadIR, "ReadIR", 512, nullptr, 2, nullptr);

    Serial.println(F("[RTOS+IR] Arrancando scheduler..."));
    vTaskStartScheduler();

    Serial.println(F("[RTOS+IR] vTaskStartScheduler() fallo (RAM insuficiente)"));
    while (true) {}
}

void loop() {}
