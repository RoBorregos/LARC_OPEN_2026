/*
*@author:  Ximena Patricia García Magdaleno
* StopState.hpp
* State Machine Tier Three.4- Stop State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class StopState {
public:
    void begin() {}

    void update() {
        LARC.stop();
    }
};
