/**
 * @file drive_statemachine_test.cpp
 * @brief Standalone diagnostic (no RTOS): runs the full DriveStateMachineTest
 *        state flow (same states/transitions as AntiqueStateMachine, driven
 *        by IR, VLX/ToF and QTR) but moving through LARC (Drive: BNO085
 *        yaw-hold PID + omni open-loop kinematics) instead of odomMove_
 *        (OdomMovement closed-loop RPM PID) -- same drive stack and BNO085
 *        init/wait sequence as src/test_sensors/testDrive.cpp.
 *
 * pio run -e drive_statemachine_test -t upload -t monitor
 */

#include <Arduino.h>
#include "robot/StateMachine/DriveStateMachineTest/DriveStateMachineTest.hpp"

DriveStateMachineTest sm;

void setup() {
  Serial.begin(115200);
  delay(500);

  // sm.begin() calls LARC.begin() internally, which waits (bounded to 1s)
  // for the first valid BNO085 yaw sample before latching the yaw-hold
  // target -- same guard as Drive::begin() in testDrive.cpp, so a dead/
  // absent BNO085 on Wire2 can't hang setup() here either.
  sm.begin();
}

void loop() {
  // Same order as testDrive.cpp: updateControl() (LARC.update() -- BNO085 +
  // yaw PID + omni mixing) applies the command set on the previous
  // iteration, then update() reads IR/VLX/QTR, runs the state logic and
  // sets the next forward/backward/left/right/setTranslation command.
  sm.updateControl();
  sm.update();
}
