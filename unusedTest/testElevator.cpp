#include <Arduino.h>
#include "Elevator.hpp"
#include "Pins.h"

Elevator elevator_;

const int limitSwitch = Pins::kLimitSwitch;
bool lastLimitState = false;

void setup()
{
    Serial.begin(115200);
    elevator_.begin();
    pinMode(limitSwitch, INPUT_PULLUP);
}

void loop()
{
    // UP
    elevator_.ElevatorPosition(1);
    delay(9000);

    // Stop
    elevator_.ElevatorPosition(0);
    delay(2000);

    // DOWN until limit switch is pressed
    elevator_.ElevatorPosition(2);

    while (true)
    {
        bool limitPressed = (digitalRead(limitSwitch) == HIGH);

        if (limitPressed != lastLimitState)
        {
            if (limitPressed)
                Serial.println("LIMIT SWITCH PRESIONADO");
            else
                Serial.println("LIMIT SWITCH LIBERADO");

            lastLimitState = limitPressed;
        }

        if (limitPressed)
        {
            elevator_.ElevatorPosition(0);
            Serial.println("ELEVADOR DETENIDO POR LIMIT SWITCH");
            break;
        }
    }

    //delay(2000); //Uncoment the delay so it goes UP again
}