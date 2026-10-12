// src/main.cpp
#include <Arduino.h>
#include "mux.h"   // tu clase del 74HC4067
#include "pins.h"

// Este code es para probar todos los canales del mux análogo
// Da como resultado la señal analoga de cada uno de los canales

Mux74HC4067 mux;

void setup() {
  Serial.begin(115200);
  delay(300);

  mux.begin();                 // configura pines y selecciona canal 0
  mux.setSigMode(INPUT);       // por claridad (tu begin ya lo hace)

  Serial.println(F("\n[MUX] Leyendo 16 canales (0-15) ..."));
  Serial.println(F("Formato: ch:valor"));
}

void loop() {
  mux.debugPrint();
}