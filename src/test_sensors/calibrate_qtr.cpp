/**
 * @file calibrate_qtr.cpp
 * @brief Calibra el QTR frontal real (instances.hpp: qtrFront, C0-C6 del
 *        mux1, ver lib/sensors/qtr/qtr.hpp -- N=7 sensores) moviendo el
 *        robot a mano sobre la linea durante 10s. Usa el mismo objeto
 *        qtrFront que despues consume la state machine, asi que los
 *        min/max impresos se pueden copiar directo a
 *        Constants::QTRCalibration::Front en constants.h.
 *
 *        Antes vivia en test/ (pio test, nunca se compilaba con pio run) --
 *        movido aqui por la misma razon que testDrive.cpp/testMotors.cpp
 *        (ver nota en env:test_drive de platformio.ini).
 *
 * pio run -e qtr_calibrate -t upload -t monitor
 */

#include <Arduino.h>
#include "pins.h"
#include "qtr.hpp"
#include "mux.h"
#include "robot/instances/instances.hpp"

void setup() {
    Serial.begin(115200);
    delay(500);
    mux.begin();
    qtrFront.begin();

    Serial.println("Calibrating FRONT move robot over line for 10s...");
    qtrFront.calibrate(10000);
    qtrFront.printCalibration("FRONT");

    Serial.println("Done — copy values to constants.h");
}

void loop() {}