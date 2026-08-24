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
        // Nada que inicializar
    }

    void update(uint32_t now) {
        // Solo detener el robot
        LARC.brake();
    }

private:
    // Sin estado interno
};