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