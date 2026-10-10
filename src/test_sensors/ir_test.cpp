/**
 * @file ir_test.cpp
 * @brief Standalone diagnostic of the 4 IR line sensors, same wiring as
 *        robot/instances/instances.cpp: L1 (FL) / L2 (FR) on direct GPIO,
 *        L3 (BL) / L4 (BR) as analog channels on the mux (kMuxSig2).
 *
 *        Prints the IRLine state used by Perception (FL FR BL BR) plus the
 *        raw mux value of BL/BR against its threshold.
 *
 * Place one corner of the robot at a time over the line and check which
 * column turns on (e.g. back-left corner only -> BL should be 1).
 *
 * pio run -e ir_test -t upload -t monitor
 */

#include <Arduino.h>

#include "pins.h"
#include "constants.h"
#include "IR/IR.hpp"
#include "mux.h"

Mux74HC4067 mux(Pins::kMuxSig2);
IRLine ir(Pins::kIrChFL, Pins::kIrChFR, mux,
          Pins::kIrChBLMux, Pins::kIrChBRMux, 0b0000);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    ir.begin();

    Serial.printf("[IR TEST] FL pin %u | FR pin %u | BL mux C%u (thr %u) | BR mux C%u (thr %u)\n",
                  Pins::kIrChFL, Pins::kIrChFR,
                  Pins::kIrChBLMux, Constants::IRCalibration::kThreshBL,
                  Pins::kIrChBRMux, Constants::IRCalibration::kThreshBR);
}

void loop()
{
    static uint32_t lastPrint = 0;

    ir.update();

    const uint32_t now = millis();
    if (now - lastPrint < 150)
        return;
    lastPrint = now;

    Serial.printf("FL=%d FR=%d BL=%d BR=%d | rawBL=%d rawBR=%d\n",
                  ir.getState(IRLine::FL), ir.getState(IRLine::FR),
                  ir.getState(IRLine::BL), ir.getState(IRLine::BR),
                  mux.read(Pins::kIrChBLMux), mux.read(Pins::kIrChBRMux));
}
