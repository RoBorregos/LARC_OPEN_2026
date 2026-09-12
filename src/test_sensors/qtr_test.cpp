/**
 * @file qtr_test.cpp
 * @brief Diagnostico standalone (sin RTOS) del QTRX-MD-08A (array de
 *        reflectancia analogico) cableado a traves del mux2 74HC4067
 *        (ver lib/sensors/mux.h e include/pins.h: kMuxSig2 / kMuxS0-S3).
 *        El QTR real esta conectado a SIG_A1 (kMuxSig2), no a SIG_A0
 *        (kMuxSig) -- confirmado porque vlx_qtr_test.cpp (que escanea
 *        kMuxSig2) veia valores variables mientras este test con kMuxSig
 *        se quedaba fijo.
 *
 *        Escanea los 16 canales (C0..C15) del mux, igual que
 *        vlx_qtr_test.cpp -- todavia no se sabe en que canal(es) exactos
 *        esta conectado el QTR fisico (ver nota en pins.h), asi que no
 *        hay que asumir un rango fijo (C0..C7) o se puede estar leyendo
 *        canales sin nada conectado mientras el sensor real esta en
 *        C8..C15.
 *
 *        Lee los canales directo via Mux74HC4067::read() -- NO usa la
 *        clase QTR (lib/sensors/qtr) porque esta hardcodeada a N=7
 *        sensores fijos empezando en un firstChannel conocido.
 *
 * Como usarlo: abre el monitor serie y pasa el array sobre blanco y negro.
 * Las columnas Cn que correspondan al QTR real deben subir/bajar de forma
 * estable con la reflectancia (mas reflectante = valor mas alto); el resto
 * de canales sin nada conectado se van a quedar planos/ruidosos. Anota los
 * min/max de los canales que sí respondan para armar la calibracion
 * despues (setCalibration en constants.h).
 *
 * Ademas de eso instancia un QTR real (mismo firstChannel/calibracion
 * "Front" que usa qtrFront en instances.cpp, pero como objeto local aqui
 * para no arrastrar todo el robot) e imprime getPosition()/onLine() --
 * eso es exactamente lo que ve la state machine, sin motores ni PID de
 * por medio. Sirve para mover el sensor a mano sobre la linea y confirmar
 * si el problema esta en la lectura/posicion o en lo que pasa despues
 * (motor/PID) en DriveStateMachineTest.
 *
 * pio run -e qtr_test -t upload -t monitor
 */

#include <Arduino.h>
#include "mux.h"
#include "qtr.hpp"
#include "constants.h"

static Mux74HC4067 mux(Pins::kMuxSig2);
static constexpr uint8_t kFirstCh = 0;  // C0..C15, todo el mux
static constexpr uint8_t kNumCh   = 16;

static uint16_t minVal[kNumCh];
static uint16_t maxVal[kNumCh];

// Mismo objeto/calibracion que usa DriveStateMachineTest (C0..C6, perfil Front)
static QTR qtrFront(0, mux);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    mux.begin();
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0); // FRONT

    for (uint8_t i = 0; i < kNumCh; ++i)
    {
        minVal[i] = 1023;
        maxVal[i] = 0;
    }

    Serial.println(F("[QTR TEST] Escaneo de 16 canales via mux 74HC4067 (SIG_A1)"));
    Serial.printf("[QTR TEST] SIG=%u S0=%u S1=%u S2=%u S3=%u, canales %u..%u\n",
                  Pins::kMuxSig2, Pins::kMuxS0, Pins::kMuxS1, Pins::kMuxS2, Pins::kMuxS3,
                  kFirstCh, kFirstCh + kNumCh - 1);
    Serial.println(F("[QTR TEST] qtrFront (C0-C6, calibracion Front): pos 0=extremo izq, 3000=centro, 6000=extremo der"));
}

void loop()
{
    uint16_t raw[kNumCh];

    for (uint8_t i = 0; i < kNumCh; ++i)
    {
        raw[i] = mux.read(kFirstCh + i);
        if (raw[i] < minVal[i]) minVal[i] = raw[i];
        if (raw[i] > maxVal[i]) maxVal[i] = raw[i];
    }

    qtrFront.update();

    static uint32_t lastPrint = 0;
    const uint32_t now = millis();
    if (now - lastPrint >= 100)
    {
        lastPrint = now;

        Serial.print(F("RAW: "));
        for (uint8_t i = 0; i < kNumCh; ++i)
            Serial.printf("C%u=%4u ", kFirstCh + i, raw[i]);

        Serial.print(F(" | min/max: "));
        for (uint8_t i = 0; i < kNumCh; ++i)
            Serial.printf("[%u,%u] ", minVal[i], maxVal[i]);

        Serial.print(F(" | qtrFront pos:")); Serial.print(qtrFront.getPosition());
        Serial.print(F(" onLine:"));         Serial.print(qtrFront.onLine());

        Serial.println();
    }
}
