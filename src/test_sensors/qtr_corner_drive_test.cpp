/**
 * @file qtr_corner_drive_test.cpp
 * @brief Diagnostico standalone (sin RTOS) de la correccion lateral de
 *        LOOKFORCORNER (ver DriveStateMachineTest::handleLookForCornerState,
 *        case 0 -- bloque comentado por ruido del QTR en motores viejos,
 *        ya arreglado con el filtro de mediana en QTR::update()).
 *
 *        Reproduce exactamente esa logica (mismas constantes kCornerSetpoint/
 *        kCornerKp/kCornerCorrMax/alpha del filtro paso-bajo) pero aislada de
 *        la maquina de estados: sin IR, sin ToF, sin Vision. Deja ver en vivo
 *        si el P + filtro corrige hacia el centro de forma suave (sin el
 *        swing violento que motivo bajar kCornerCorrMax a 0.20, ver comentario
 *        en DriveStateMachineTest.cpp) antes de descomentar el bloque real.
 *
 *        El QTR usado es el mismo objeto/calibracion que qtrFront en
 *        instances.cpp (C0-C6 del mux2/kMuxSig2, perfil "Front") pero como
 *        instancia local aqui, igual que qtr_test.cpp.
 *
 *        El robot SI se mueve: LARC.setTranslation(-0.20f, -corr) real via
 *        Drive (BNO085/Wire2 yaw-hold + omni open-loop) -- probar con el
 *        robot sobre un soporte o con espacio libre atras.
 *
 * Como usarlo: abre el monitor serie y pasa el QTR sobre la linea, moviendolo
 * de lado a lado. corrTarget/corrFilt deben crecer hacia +-kCornerCorrMax
 * cuando el error crece y volver a 0 cerca de kCornerSetpoint. Si se pierde
 * la linea (onLine=0), corrTarget se congela en 0 (no en el ultimo valor) y
 * el filtro decae hacia 0 en vez de seguir empujando con un error viejo.
 *
 * pio run -e qtr_corner_test -t upload -t monitor
 */

#include <Arduino.h>
#include "mux.h"
#include "qtr.hpp"
#include "constants.h"
#include "subsystem/Drive/Drive.hpp"

static Mux74HC4067 mux(Pins::kMuxSig2);
static QTR qtrFront(0, mux);
static Drive drive;

// Mismos valores que el bloque comentado en
// DriveStateMachineTest.cpp::handleLookForCornerState (case 0). Si se
// afinan alla, replicar aqui para que el diagnostico siga representando
// lo mismo que se probaria en la maquina de estados real.
static constexpr float kCornerSetpoint  = 2900.0f;
static constexpr float kCornerKp        = 0.00012f;
static constexpr float kCornerCorrMax   = 0.20f;
static constexpr float kCornerCorrAlpha = 0.15f;

// vx de LARC.setTranslation(-0.20f, -corr) en el estado real (retrocede
// mientras corrige lateral).
static constexpr float kCornerVx = -0.20f;

static float cornerCorrFiltered = 0.0f;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    mux.begin();
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0); // FRONT

    Serial.println(F("[QTR CORNER TEST] Correccion lateral de LOOKFORCORNER, aislada"));
    Serial.printf("[QTR CORNER TEST] setpoint=%.0f Kp=%.5f corrMax=%.2f alpha=%.2f vx=%.2f\n",
                  kCornerSetpoint, kCornerKp, kCornerCorrMax, kCornerCorrAlpha, kCornerVx);

    drive.begin();
    drive.holdYaw(true);
    drive.setTargetYaw(drive.getYaw());
}

void loop()
{
    qtrFront.update();

    const bool onLine = qtrFront.onLine();
    const int  lPos    = qtrFront.getPosition();

    // Igual que el bloque comentado: si no ve linea, el target se congela
    // en 0 (no se recalcula con lPos viejo) para que el filtro decaiga a 0
    // en vez de seguir empujando hacia el ultimo lado visto.
    float corrTarget = 0.0f;
    if (onLine)
    {
        const float error = kCornerSetpoint - lPos;
        corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
    }

    cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

    drive.setTranslation(kCornerVx, -cornerCorrFiltered);

    static uint32_t lastPrint = 0;
    const uint32_t now = millis();
    if (now - lastPrint >= 100)
    {
        lastPrint = now;

        const uint16_t* raw  = qtrFront.getRaw();
        const uint16_t* norm = qtrFront.getNorm();

        Serial.print(F("onLine:")); Serial.print(onLine);
        Serial.print(F(" lPos:"));  Serial.print(lPos);
        Serial.print(F(" corrTarget:")); Serial.print(corrTarget, 4);
        Serial.print(F(" corrFilt:"));   Serial.print(cornerCorrFiltered, 4);
        Serial.print(F(" vx:")); Serial.print(kCornerVx, 2);
        Serial.print(F(" vy:")); Serial.print(-cornerCorrFiltered, 4);

        Serial.print(F(" | raw:"));
        for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(raw[i]); Serial.print(','); }
        Serial.print(F(" norm:"));
        for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(norm[i]); Serial.print(','); }

        Serial.println();
    }

    // Drive::update() se auto-limita a kControlMs (10ms) internamente, no
    // hace falta bloquear el loop (ver Drive.hpp).
    drive.update();
}
