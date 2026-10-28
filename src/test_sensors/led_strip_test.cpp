/**
 * @file led_strip_test.cpp
 * @brief Cycles the 8 NeoPixel LEDs on Pins::kLedStripPin through red, green,
 * blue and white, re-sending the frame every second so the strip still lights
 * if it gets power after the Teensy boots.
 * pio run -e led_strip_test -t upload -t monitor
 */


#include <Adafruit_NeoPixel.h>

#define LED_PIN 41
#define NUM_LEDS 8

Adafruit_NeoPixel strip(
  NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800
);

void setup() {
  strip.begin();
  strip.setBrightness(10);
  strip.clear();
  strip.show();
}

void loop() {
  strip.fill(strip.Color(255, 255, 255)); // blanco
  strip.show();
  delay(200);
}