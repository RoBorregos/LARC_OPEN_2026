#include <Arduino.h>

uint32_t lastTime = 0;

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600);

    delay(1000);
    Serial.println("HC-05 test with Teensy");
}

void loop()
{
    if (millis() - lastTime > 1000)
    {
        lastTime = millis();
        Serial1.println("desde teensy");
        Serial.println("Mensaje que se envia al ~~~~ HC-05 ~~~~ ");
    }

    if (Serial1.available())
    {
        char c = Serial1.read();
        Serial.write(c);
    }
}