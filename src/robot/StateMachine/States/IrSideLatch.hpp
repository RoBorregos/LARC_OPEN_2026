/*
*@author:  Ximena Patricia García Magdaleno
* IrSideLatch.hpp
* One side's IR pair: first IR latches, the other IR of the same side triggers the correction.
*/
#pragma once
#include <Arduino.h>

struct IrSideLatch {
    static constexpr uint32_t kLatchTimeoutMs = 800;
    static constexpr uint32_t kLatchIrOffMs   = 150;

    bool     latched    = false;
    bool     firstFront = false;
    uint32_t latchMs    = 0;
    uint32_t offSinceMs = 0;

    void clear() {
        latched    = false;
        latchMs    = 0;
        offSinceMs = 0;
    }

    bool update(uint32_t now, bool front, bool back) {
        if (!latched) {
            if (front && back)
                return true;
            if (front || back) {
                latched    = true;
                firstFront = front;
                latchMs    = now;
                offSinceMs = 0;
            }
            return false;
        }

        const bool firstOn = firstFront ? front : back;
        const bool otherOn = firstFront ? back : front;

        if (otherOn) {
            clear();
            return true;
        }
        if ((now - latchMs) >= kLatchTimeoutMs) {
            clear();
            return false;
        }
        if (firstOn) {
            offSinceMs = 0;
        } else {
            if (offSinceMs == 0)
                offSinceMs = now;
            if ((now - offSinceMs) >= kLatchIrOffMs)
                clear();
        }
        return false;
    }
};
