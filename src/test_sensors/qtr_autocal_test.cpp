/**
 * @file qtr_autocal_test.cpp
 * @brief Prueba standalone (sin RTOS, sin motores) de la autocalibracion de
 *        qtrFront y qtrRear (ver QTR::ambientCalibrate / beginAutoCal /
 *        endAutoCal en lib/sensors/qtr/qtr.hpp). Mismos canales y mismo mux
 *        que instances.cpp (Pins::kMuxSig2, kQtrFrontFirstCh/kQtrRearFirstCh)
 *        pero con objetos locales para no arrastrar todo el robot.
 *
 * Orden de ejecucion (el mismo que se usaria en la state machine):
 *   1) setup(): carga los perfiles de constants.h (Front / Rear).
 *   2) setup(): cuenta regresiva de 3 s y ambientCalibrate() en AMBOS QTR con
 *      el robot QUIETO sobre fondo sin linea. Corrige por luz ambiente.
 *   3) loop(): comandos por serial (115200):
 *        l  -> abre la ventana de aprendizaje en ambos QTR. Mueve el robot a
 *              mano (o con motores) cruzando/siguiendo la linea.
 *        e  -> cierra la ventana, aplica lo aprendido e imprime los valores
 *              listos para copiar a Constants::QTRCalibration en constants.h.
 *        a  -> repite el paso 2 (ambient) con el robot quieto sobre fondo.
 *        d  -> vuelve a los defaults de constants.h.
 *
 * pio run -e qtr_autocal_test -t upload -t monitor
 */

#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "qtr.hpp"
#include "constants.h"

static Mux74HC4067 mux(Pins::kMuxSig2);
static QTR qtrFront(Pins::kQtrFrontFirstCh, mux);
static QTR qtrRear(Pins::kQtrRearFirstCh, mux);

static void loadDefaults()
{
    qtrFront.useDefaultCalibration(0); // FRONT
    qtrRear.useDefaultCalibration(1);  // REAR
}

static void printBoth(const char* title)
{
    Serial.println(title);
    qtrFront.printCalibration("FRONT");
    qtrRear.printCalibration("REAR");
}

static void runAmbient()
{
    Serial.println(F("[AUTOCAL] Ambient: robot QUIETO sobre fondo SIN linea. Empieza en 3 s..."));
    delay(3000);

    const bool okF = qtrFront.ambientCalibrate(300);
    const bool okR = qtrRear.ambientCalibrate(300);

    Serial.print(F("[AUTOCAL] ambient FRONT: "));
    if (okF) { Serial.print(F("OK, delta calMin = ")); Serial.println(qtrFront.getAmbientDelta()); }
    else     { Serial.println(F("RECHAZADO (algun sensor ve linea), se conservan los valores previos")); }

    Serial.print(F("[AUTOCAL] ambient REAR : "));
    if (okR) { Serial.print(F("OK, delta calMin = ")); Serial.println(qtrRear.getAmbientDelta()); }
    else     { Serial.println(F("RECHAZADO (algun sensor ve linea), se conservan los valores previos")); }

    printBoth("[AUTOCAL] calibracion tras ambient:");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    mux.begin();
    qtrFront.begin();
    qtrRear.begin();

    loadDefaults();
    printBoth("[AUTOCAL] defaults de constants.h:");

    runAmbient();

    Serial.println(F("[AUTOCAL] Comandos: l=abrir ventana  e=cerrar y aplicar  a=ambient  d=defaults"));
}

void loop()
{
    qtrFront.update();
    qtrRear.update();

    while (Serial.available())
    {
        const char c = (char)Serial.read();
        switch (c)
        {
        case 'l':
            qtrFront.beginAutoCal();
            qtrRear.beginAutoCal();
            Serial.println(F("[AUTOCAL] Ventana ABIERTA -- mueve el robot sobre la linea, luego 'e'"));
            break;

        case 'e':
        {
            const uint8_t nF = qtrFront.endAutoCal();
            const uint8_t nR = qtrRear.endAutoCal();
            Serial.print(F("[AUTOCAL] Ventana cerrada. Sensores cambiados: FRONT="));
            Serial.print(nF);
            Serial.print(F("/7 REAR="));
            Serial.print(nR);
            Serial.println(F("/7"));
            printBoth("[AUTOCAL] copiar a constants.h si se ven razonables:");
            break;
        }

        case 'a':
            runAmbient();
            break;

        case 'd':
            loadDefaults();
            printBoth("[AUTOCAL] defaults restaurados:");
            break;

        default:
            break;
        }
    }

    static uint32_t lastPrint = 0;
    const uint32_t now = millis();
    if (now - lastPrint >= 200)
    {
        lastPrint = now;

        Serial.print(F("F pos:")); Serial.print(qtrFront.getPosition());
        Serial.print(F(" on:"));   Serial.print(qtrFront.onLine());
        Serial.print(F(" | R pos:")); Serial.print(qtrRear.getPosition());
        Serial.print(F(" on:"));      Serial.print(qtrRear.onLine());
        Serial.print(F(" | learn:")); Serial.println(qtrFront.isAutoCalActive());
    }
}
