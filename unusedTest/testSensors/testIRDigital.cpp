#include <Arduino.h>
#include "IR_digitales/IR_digital.hpp"

const uint8_t irPins[IR_digital::N] = {
    13, // FL
    12, // FR
    11, // BL
    10  // BR
};

// Cambia la máscara si algun sensor sale invertido
IR_digital ir(irPins, 0b0000);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting IR digital test...");
    ir.begin();
}

void loop()
{
    ir.update();

    bool fl = ir.getState(IR_digital::FL);
    bool fr = ir.getState(IR_digital::FR);
    bool bl = ir.getState(IR_digital::BL);
    bool br = ir.getState(IR_digital::BR);

    Serial.print("FL: ");
    Serial.print(fl);
    Serial.print(" | FR: ");
    Serial.print(fr);
    Serial.print(" | BL: ");
    Serial.print(bl);
    Serial.print(" | BR: ");
    Serial.println(br);

    delay(100);
}