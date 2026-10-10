/*
*@author:  Ximena Patricia García Magdaleno
* BeansGoBack.hpp
* State Machine Tier Two.3- Beans Go Back State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"

class BeansGoBackState {
public:
    void begin() {
        action_stage = 0;
    }

    void update(bool BL, bool BR, bool FL) {
        // No bajar el elevador ::

        switch (action_stage) {
            // ── Stage 0: retroceder hasta encontrar BR o BL ──────────────────────
            case 0:
                vision.stop();
                vision.clearErrors();
                elevator.ElevatorPosition(0); //zero for stop

                if (BR || BL) {
                    LARC.stop();
                    action_stage = 1;
                    return;
                }

                LARC.backward(0.30f);
                return;

            // ── Stage 1: LARC.left hasta encontrar FL ────────────────────────────
            case 1:
                elevator.ElevatorPosition(0);

                if (FL) {
                    LARC.stop();
                    action_stage = 2;
                    return;
                }

                LARC.left(0.30f);
                return;

            // ── Stage 2: detenido ─────────────────────────────────────────────────
            case 2:
                elevator.ElevatorPosition(0);
                LARC.stop();
                return;
        }
    }

private:
    int action_stage = 0;
};
