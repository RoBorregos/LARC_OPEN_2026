/*
    Prueba standalone (sin RTOS) de la clase Elevator. Pines via Pins::
    (pins.h): IN1_M5 = pin 4, IN2_M5 = pin 3, PWM_M5 = pin 12 (velocidad
    fija en Elevator.cpp).

    Elevator::ElevatorPosition(): 0 = stop, 1 = subir, 2 = bajar.

    OJO: esta prueba no lee los limit switches. Por eso subir/bajar se cortan solos a los
    kMoveMs para no chocar contra el tope mecanico.

    Comandos por serial (115200):
      u -> subir (se detiene solo a los kMoveMs)
      d -> bajar (se detiene solo a los kMoveMs)
      s -> parar ya

    pio run -e test_elevator -t upload -t monitor
*/

#include <Arduino.h>
#include <Wire.h>
#include "Elevator.hpp"

namespace
{
    constexpr uint32_t kMoveMs = 900; // corte de seguridad por movimiento

    constexpr int kStop = 0;
    constexpr int kUp   = 1;
    constexpr int kDown = 2;

    Elevator elevator;

    int      currentState = kStop;
    uint32_t stateSince   = 0;

    void setState(int s)
    {
        currentState = s;
        stateSince   = millis();
        elevator.ElevatorPosition(s);
        Serial.println(s == kUp ? "[elevator] UP" : (s == kDown ? "[elevator] DOWN" : "[elevator] STOP"));
    }
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 900) {}

    elevator.begin();
    setState(kStop);

    Serial.println("Elevator test (sin control de velocidad): u=subir  d=bajar  s=stop");
}

void loop()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == 'u')      setState(kUp);
        else if (c == 'd') setState(kDown);
        else if (c == 's') setState(kStop);
    }

    if (currentState != kStop && millis() - stateSince >= kMoveMs)
        setState(kStop);
}
