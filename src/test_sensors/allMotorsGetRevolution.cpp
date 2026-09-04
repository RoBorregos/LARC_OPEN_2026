/*
- Ticks are summed to obtain revolutions
*/
#include <Arduino.h>
#include "pins.h"

// Channel B (direction)
const uint8_t encB_UL = Pins::kEncoders[0];
const uint8_t encB_UR = Pins::kEncoders[3];
const uint8_t encB_LL = Pins::kEncoders[5];
const uint8_t encB_LR = Pins::kEncoders[7];

// Channel A (interrupts)
const uint8_t encA_UL = Pins::kEncoders[1];
const uint8_t encA_UR = Pins::kEncoders[2];
const uint8_t encA_LL = Pins::kEncoders[4];
const uint8_t encA_LR = Pins::kEncoders[6];

volatile long ticksUL = 0;
volatile long ticksUR = 0;
volatile long ticksLL = 0;
volatile long ticksLR = 0;

void isrUL() { ticksUL++; }
void isrUR() { ticksUR++; }
void isrLL() { ticksLL++; }
void isrLR() { ticksLR++; }

void setup()
{
    Serial.begin(115200);

    pinMode(encA_UL, INPUT_PULLUP); pinMode(encB_UL, INPUT_PULLUP);
    pinMode(encA_UR, INPUT_PULLUP); pinMode(encB_UR, INPUT_PULLUP);
    pinMode(encA_LL, INPUT_PULLUP); pinMode(encB_LL, INPUT_PULLUP);
    pinMode(encA_LR, INPUT_PULLUP); pinMode(encB_LR, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encA_UL), isrUL, RISING);
    attachInterrupt(digitalPinToInterrupt(encA_UR), isrUR, RISING);
    attachInterrupt(digitalPinToInterrupt(encA_LL), isrLL, RISING);
    attachInterrupt(digitalPinToInterrupt(encA_LR), isrLR, RISING);

    Serial.println("Listo. Gira el motor y observa los ticks.");
    Serial.println("Envia 'r' por Serial para resetear contadores.");
}

void loop()
{
    // Reset with 'r' from the serial monitor
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 'r' || c == 'R')
        {
            noInterrupts();
            ticksUL = ticksUR = ticksLL = ticksLR = 0;
            interrupts();
            Serial.println(">>> Contadores reseteados <<<");
        }
    }

    // Print every 200ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();

        noInterrupts();
        long ul = ticksUL;
        long ur = ticksUR;     //471   ||  482  ||   484
        long ll = ticksLL;     // 495  ||  496
        long lr = ticksLR;
        interrupts();

        Serial.print("UL: "); Serial.print(ul);
        Serial.print(" | UR: "); Serial.print(ur);
        Serial.print(" | LL: "); Serial.print(ll);
        Serial.print(" | LR: "); Serial.println(lr);
    }
}