/**
 * @file blink_test.cpp
 * @brief Test program to blink the built-in LED on the board. Confirm that
 * the board is working and that the Arduino core is functioning correctly.
 * pio run -e blink_test -t upload
 */

#include <Arduino.h>

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
}
