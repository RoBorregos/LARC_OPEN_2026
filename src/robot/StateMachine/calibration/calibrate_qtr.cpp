/**
 * @file calibrate_qtr.cpp
 * @brief Calibra los QTR frontal y trasero reales (instances.hpp: qtrFront,
 *        qtrRear, C0-C6, ver lib/sensors/qtr/qtr.hpp -- N=7 sensores)
 *        moviendo el robot a mano sobre la linea durante 10s cada uno. Usa
 *        los mismos objetos qtrFront/qtrRear que despues consume la state
 *        machine, asi que los min/max impresos se pueden copiar directo a
 *        Constants::QTRCalibration::Front / ::Rear en constants.h.
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
    qtrRear.begin();

    Serial.println("Calibrating FRONT move robot over line for 10s...");
    qtrFront.calibrate(10000);
    qtrFront.printCalibration("FRONT");

    //Serial.println("Calibrating REAR move robot over line for 10s...");
    //qtrRear.calibrate(10000);
    //qtrRear.printCalibration("REAR");

    Serial.println("Done — copy values to constants.h");
}

void loop() {}