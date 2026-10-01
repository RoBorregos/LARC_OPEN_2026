/*
    Standalone elevator test with two limit switches.
    - Limit1 (Pins::kLimitSwitch) pressed  -> elevator goes DOWN
    - Limit1 (Pins::kLimitSwitch) released -> elevator goes UP
    - Limit2 (Pins::kLimitSwitch2) is only read and printed.

    pio run -e test_elevator -t upload -t monitor
*/

#include <Arduino.h>
#include "Elevator.hpp"
#include "pins.h"

namespace
{
    // Switches have external pull-ups, so plain INPUT (no internal pull-up).
    // Flip to LOW if the switches read inverted.
    constexpr int kPressedLevel = HIGH;

    constexpr int kStop = 0;
    constexpr int kUp   = 1;
    constexpr int kDown = 2;

    Elevator elevator;

    int  currentState = -1;
    bool lastLimit1   = false;
    bool lastLimit2   = false;

    bool isPressed(uint8_t pin) { return digitalRead(pin) == kPressedLevel; }

    void setState(int s)
    {
        if (s == currentState)
            return;
        currentState = s;
        elevator.ElevatorPosition(s);
        Serial.println(s == kUp ? "[elevator] UP" : (s == kDown ? "[elevator] DOWN" : "[elevator] STOP"));
    }
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 900) {}

    pinMode(Pins::kLimitSwitch, INPUT);
    pinMode(Pins::kLimitSwitch2, INPUT);

    elevator.begin();
    setState(kStop);

    Serial.println("Elevator test: Limit1 pressed = DOWN, released = UP");
}

void loop()
{
    const bool limit1 = isPressed(Pins::kLimitSwitch);
    const bool limit2 = isPressed(Pins::kLimitSwitch2);

    if (limit1 != lastLimit1)
    {
        Serial.println(limit1 ? "Limit1 PRESSED" : "Limit1 RELEASED");
        lastLimit1 = limit1;
    }
    if (limit2 != lastLimit2)
    {
        Serial.println(limit2 ? "Limit2 PRESSED" : "Limit2 RELEASED");
        lastLimit2 = limit2;
    }

    setState(limit1 ? kDown : kUp);
}
