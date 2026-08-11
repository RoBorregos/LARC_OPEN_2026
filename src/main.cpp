#include <Arduino.h>
#include "PeriodicRunner.hpp"
#include "robot/StateMachine/StateMachine.hpp"
#include "robot/instances/instances.hpp"

LARCStateMachine stateMachine;

namespace
{
    // These periods become vTaskDelayUntil() periods during the RTOS phase.
    PeriodicRunner driveControlSchedule(10);
    PeriodicRunner robotSchedule(20);
    PeriodicRunner actuatorSchedule(20);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    ir.begin();
    qtrFront.begin();
    servos.begin();
    elevator.begin();

    stateMachine.begin();
}

void loop()
{
    const uint32_t now = millis();

    if (driveControlSchedule.due(now))
        stateMachine.updateControl();

    if (robotSchedule.due(now))
        stateMachine.update();

    if (actuatorSchedule.due(now))
        servos.update();
}
