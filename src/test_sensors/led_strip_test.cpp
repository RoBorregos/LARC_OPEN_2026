/**
 * @file led_strip_test.cpp
 * @brief Cycles the 8 NeoPixel LEDs on Pins::kLedStripPin through red, green,
 * blue and white, re-sending the frame every second so the strip still lights
 * if it gets power after the Teensy boots.
 * pio run -e led_strip_test -t upload -t monitor
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"

static constexpr uint16_t kNumLeds = 8;
static constexpr uint32_t kStepMs  = 1000;

Adafruit_NeoPixel leds(kNumLeds, 15, NEO_GRB + NEO_KHZ800);

struct Step
{
    const char *name;
    uint8_t r, g, b;
};

static const Step kSteps[] = {
    {"ROJO", 255, 0, 0},
    {"VERDE", 0, 255, 0},
    {"AZUL", 0, 0, 255},
    {"BLANCO", 255, 255, 255},
};
static constexpr uint8_t kNumSteps = sizeof(kSteps) / sizeof(kSteps[0]);

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
    {
    }

    leds.begin();
    leds.setBrightness(30);

    Serial.print("LED strip test en pin ");
    Serial.println(Pins::kLedStripPin);
}

void loop()
{
    static uint8_t step = 0;

    const Step &s = kSteps[step];
    leds.fill(leds.Color(s.r, s.g, s.b));
    leds.show();

    Serial.print("Color: ");
    Serial.println(s.name);

    step = (step + 1) % kNumSteps;
    delay(kStepMs);
}
