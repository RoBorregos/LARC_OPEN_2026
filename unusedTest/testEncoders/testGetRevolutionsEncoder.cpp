/*
 * Encoder Revolution and RPM Test
 *
 * This code reads the signals from four quadrature encoders connected
 * to the robot's motors: Upper Left (UL), Upper Right (UR),
 * Lower Left (LL), and Lower Right (LR).
 *
 * Interrupts are used to count the encoder ticks each time a rising
 * edge is detected on channel A of each encoder.
 *
 * For the Upper Left motor, channel B is also read to determine
 * the direction of rotation and increase or decrease the tick count.
 *
 * The code calculates the RPM of the Upper Left motor using:
 *
 * RPM = (delta ticks / pulses per revolution) * (60 / sample time)
 *
 * The calculated RPM is displayed through the Serial Monitor.
 *
 * PPR: 475 pulses per revolution
 * Sample time: 0.01 seconds
 */

// Code 1: obtain the revolutions

#include <Arduino.h>
#include "pins.h"

// Encoders B
const uint8_t encUL_Pin = Pins::kEncoders[0];
const uint8_t encUR_Pin = Pins::kEncoders[3];
const uint8_t encLL_Pin = Pins::kEncoders[5];
const uint8_t encLR_Pin = Pins::kEncoders[7];

// Encoders A
const uint8_t encUL_A_Pin = Pins::kEncoders[1];
const uint8_t encUR_A_Pin = Pins::kEncoders[2];
const uint8_t encLL_A_Pin = Pins::kEncoders[4];
const uint8_t encLR_A_Pin = Pins::kEncoders[6];

volatile long ticksUL = 0;
volatile long ticksUR = 0;
volatile long ticksLL = 0;
volatile long ticksLR = 0;

// RPM for one motor
long last_ticks = 0;
float rpm = 0.0f;

const float PPR = 475.0f;
const float Ts = 0.01f;

unsigned long last_time = 0;

void isrUL()
{
    if (digitalRead(encUL_Pin) == HIGH)
        ticksUL++;
    else
        ticksUL--;
    
    // 445 // 2376 (5 revolutions) = 475 <---- 
    // (445 + 475) 
    // 2 = 460 
    // 482
}

void isrUR()
{
    ticksUR++;
}

void isrLL()
{
    ticksLL++;
}

void isrLR()
{
    ticksLR++;
}

void setup()
{
    Serial.begin(115200);

    pinMode(encUL_Pin, INPUT_PULLUP);
    pinMode(encUR_Pin, INPUT_PULLUP);
    pinMode(encLL_Pin, INPUT_PULLUP);
    pinMode(encLR_Pin, INPUT_PULLUP);

    pinMode(encUL_A_Pin, INPUT_PULLUP);
    pinMode(encUR_A_Pin, INPUT_PULLUP);
    pinMode(encLL_A_Pin, INPUT_PULLUP);
    pinMode(encLR_A_Pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encUL_A_Pin), isrUL, RISING);
    attachInterrupt(digitalPinToInterrupt(encUR_A_Pin), isrUR, RISING);
    attachInterrupt(digitalPinToInterrupt(encLL_A_Pin), isrLL, RISING);
    attachInterrupt(digitalPinToInterrupt(encLR_A_Pin), isrLR, RISING);
}

void loop()
{
    if (millis() - last_time >= Ts * 1000)
    {
        //noInterrupts();
        long current_ticks = ticksUL;
        //interrupts();

        long delta_ticks = current_ticks - last_ticks;
        last_ticks = current_ticks;

        rpm = (delta_ticks / PPR) * (60.0f / Ts);

        Serial.print("RPM UL: ");
        Serial.println(rpm);

        last_time = millis();
    }

    static uint32_t lastPrint = 0;

    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        noInterrupts();
        long ul = ticksUL;
        long ur = ticksUR;
        long ll = ticksLL;
        long lr = ticksLR;
        interrupts();

        /*
        Serial.print("UL: ");
        Serial.print(ul);
        Serial.print(" | UR: ");
        Serial.print(ur);
        Serial.print(" | LL: ");
        Serial.print(ll);
        Serial.print(" | LR: ");
        Serial.println(lr);
        */
    }
}

