/**
 * @file ir_test.cpp
 * @brief Diagnostico standalone de los 4 sensores IR digitales L1-L4,
 *        usando los pines del esquematico de la PCB nueva (compartido por
 *        el equipo), NO los de include/pins.h -- esos siguen reflejando
 *        una placa anterior y no coinciden con esta (ver comparacion
 *        hecha en la sesion de debug). No pasa por ningun mux ni por la
 *        maquina de estados: es puro digitalRead() sobre los 4 pines.
 *
 *        L1 -> pin 15
 *        L2 -> pin 14
 *        L3 -> pin 23
 *        L4 -> pin 41
 *
 * Como usarlo: abre el monitor serie y pasa la mano sobre cada sensor uno
 * por uno. Un sensor conectado y funcionando debe mostrar su columna
 * cambiando de forma estable (0/1) al bloquear/destapar el sensor. Un pin
 * flotante (sensor desconectado, sin pull-up/pull-down) tipicamente se ve
 * "ruidoso": cambia solo, rapido, sin que nadie toque nada.
 *
 * pio run -e ir_test -t upload -t monitor
 */

#include <Arduino.h>

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

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    for (const auto &ir : kIrPins)
        pinMode(ir.pin, INPUT);

    Serial.println(F("[IR TEST] pines (del esquematico nuevo, sin pull-up interna):"));
    for (const auto &ir : kIrPins)
        Serial.printf("  %s -> pin %u\n", ir.label, ir.pin);
    Serial.println(F("[IR TEST] pasa la mano sobre cada sensor y observa que columna cambia."));
}

void loop()
{
    static uint32_t lastPrint = 0;
    static uint32_t sampleCount = 0;
    static uint32_t transitions[4] = {0, 0, 0, 0};
    static int lastValue[4] = {-1, -1, -1, -1};

    // Muestreo rapido para detectar "ruido" de pines flotantes entre prints.
    for (uint8_t i = 0; i < 4; ++i)
    {
        int v = digitalRead(kIrPins[i].pin);
        if (lastValue[i] != -1 && v != lastValue[i])
            ++transitions[i];
        lastValue[i] = v;
    }
    ++sampleCount;

    const uint32_t now = millis();
    if (now - lastPrint >= 200)
    {
        lastPrint = now;
        Serial.printf(
            "[%lu ms] L1=%d(%lu) L2=%d(%lu) L3=%d(%lu) L4=%d(%lu)  (valor actual(transiciones desde el print anterior, de %lu muestras))\n",
            (unsigned long)now,
            lastValue[0], (unsigned long)transitions[0],
            lastValue[1], (unsigned long)transitions[1],
            lastValue[2], (unsigned long)transitions[2],
            lastValue[3], (unsigned long)transitions[3],
            (unsigned long)sampleCount);

        for (uint8_t i = 0; i < 4; ++i)
            transitions[i] = 0;
        sampleCount = 0;
    }
}
