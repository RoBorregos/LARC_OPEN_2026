/*
*@author:  Ximena Patricia García Magdaleno
* StartState.hpp
* State Machine Tier One.3- Start State 
*/

#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class StopState {
public:
    void begin() {
        // Release whichever benefit door the Orin was holding and shut both,
        // then tell it to quit. closeBenefit() moves the servos; leave() is
        // stop() + resetGuards(), the pair every phase exit needs.
        vision.closeBenefit();
        vision.leave();
    }

    void update(uint32_t now) {
        // Solo detener el robot
        LARC.brake();
    }

private:
    // Sin estado interno
};