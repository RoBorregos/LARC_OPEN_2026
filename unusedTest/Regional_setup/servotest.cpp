
#include <Arduino.h>

#include "mux.h" // Mux74HC4067
#include "IR_mux.hpp" // IR_mux
#include "ServoSystem.hpp"
#include "pins.h"

// Use your mux pins from pins.h  
Mux74HC4067 mux(Pins::kMuxSig, Pins::kMuxS0, Pins::kMuxS1, Pins::kMuxS2, Pins::kMuxS3);

static const uint8_t IR_CH[IR_mux::N] = {
    12, // FL  -> CH12  (usado para el LOWER)
    11, // FR  -> CH11  (usado para el UPPER)
    0,  // BL  (sin usar)
    1   // BR  (sin usar)
};

IR_mux ir(mux, IR_CH, 0b0000);

ServoSystem servos;

static uint32_t lastPrint = 0;

void setup()
{
    Serial.begin(115200);
    delay(300);

    mux.begin();
    ir.begin();

    servos.begin();

    Serial.println("\n[TEST] IR CH11-Upper, CH12-Lower (deploy on line, else home)");
}

void loop()
{
    ir.update();

    //Upper servo IR FR (CH11)
    const bool ch11_line = ir.getState(IR_mux::FR);
    if (!servos.intakeUpperBusy())
    {
        if (ch11_line) servos.intakeUpperHome();
        else           servos.intakeUpperDeploy();
    }

    // --- Lower servo controlled by IR FL (CH12) ---
    const bool ch12_line = ir.getState(IR_mux::FL);
    if (!servos.intakeLowerBusy())
    {
        if (ch12_line) servos.intakeLowerHome();
        else           servos.intakeLowerDeploy();
    }

    // Debug print
    if (millis() - lastPrint >= 80)
    {
        lastPrint = millis();
        Serial.print("CH11(FR) line=");
        Serial.print(ch11_line ? 1 : 0);
        Serial.print(" raw=");
        Serial.print(ir.getRaw(IR_mux::FR));

        Serial.print(" | CH12(FL) line=");
        Serial.print(ch12_line ? 1 : 0);
        Serial.print(" raw=");
        Serial.println(ir.getRaw(IR_mux::FL));
    }
}