#include <Arduino.h>
#include "pins.h"

#include "ultrasonic/ultrasonic.hpp"
#include "subsystem/Drive/Drive.hpp"

//Funciono :)      -> puede mejerorarse
Drive drive;

// 2 ultrasonicos
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

enum class State { FORWARD, RIGHT };
State state = State::FORWARD;
uint32_t stateStart = 0;

// Arriba, global:
static uint32_t clearStartMs = 0; //limpiar y reiniciar conteo
static constexpr uint32_t kClearDelayMs = 300; // espera 300 ms sin obstaculo antes de salir de RIGHT

static uint32_t velocity = 0.45f; 


void setup()
{
  drive.begin();
  drive.holdYaw(true);

  us1.begin();
  us2.begin();

  state = State::FORWARD;
  stateStart = millis();
}

void loop()
{
 
  drive.update(); //control update
  us1.update();
  us2.update();


  float d1 = us1.getdistance();
  float d2 = us2.getdistance();

  bool obstacle =
    (us1.isValid() && d1 < 15.0f) ||
    (us2.isValid() && d2 < 15.0f);


  const uint32_t now = millis();

  // Logica
  if (state == State::FORWARD)
{
  drive.forward(velocity); //0.35f antes (chasis viejito)

  if (obstacle)
  {
    state = State::RIGHT;
    clearStartMs = 0; 
  }
}
else // RIGHT
{
  drive.left(velocity);

  if (obstacle)
  {
    // Sigue habiendo obstaculo -> reinicia
    clearStartMs = 0;
  }
  else
  {
    // Ya NO hay obstaculo: sigue con el conteo
    if (clearStartMs == 0) clearStartMs = now;

    if (now - clearStartMs >= kClearDelayMs)
    {
      state = State::FORWARD;
      clearStartMs = 0;
    }
  }
}


}