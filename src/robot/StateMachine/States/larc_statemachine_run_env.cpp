/**
 * @file larc_statemachine_run_env.cpp
 * @brief Runs LARCStateMachine (one file per state under StateMachine/States)
 *        without RTOS, same setup()/loop() as drive_statemachine_test.cpp.
 *
 * pio run -e larc_statemachine_test -t upload -t monitor
 * 
 *  .cpp File to run the StateMachine ones that are separated in State::Tires
 */

#include <Arduino.h>
#include "robot/StateMachine/StateMachine.hpp"
#include "BNO085/BNO085.hpp"

LARCStateMachine sm;

void setup() {
  BNO085::holdInReset(); // BNO in reset until LARC.begin() -> BNO085::begin() releases it
  Serial.begin(115200);
  delay(500);

  sm.begin();
}

void loop() {
  sm.updateControl();
  sm.update();
}
