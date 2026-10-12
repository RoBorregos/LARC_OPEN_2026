/**
 * @file larc_statemachine_test.cpp
 * @brief Runs LARCStateMachine (one file per state under StateMachine/States)
 *        without RTOS, same setup()/loop() as drive_statemachine_test.cpp.
 *
 * pio run -e larc_statemachine_test -t upload -t monitor
 * 
 *  .cpp File to run the StateMachine ones that are separated in State::Tires
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
