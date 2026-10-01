#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class StopState {
public:
    void begin() {
    }

    void update(uint32_t now) {
        LARC.brake();
    }

private:
};