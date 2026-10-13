/**
 * @file main.cpp
 * @brief Runs LARCStateMachine without RTOS (same as larc_statemachine_run_env.cpp).
 *
 * pio run -e teensy41 -t upload -t monitor
 */

#include <Arduino.h>
#include "robot/StateMachine/StateMachine.hpp"

LARCStateMachine sm;

void setup() {
  Serial.begin(115200);
  delay(500);

  sm.begin();
}

void loop() {
  sm.updateControl();
  sm.update();
}
