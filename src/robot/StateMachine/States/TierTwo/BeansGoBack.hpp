/*
*@author:  Ximena Patricia García Magdaleno
* BeansGoBack.hpp
* State Machine Tier Two.3- Beans Go Back State
*/
#pragma once
#include <Arduino.h>
#include "robot/instances/instances.hpp"
#include "../StateCommon.hpp"

class BeansGoBackState {
public:
    void begin() {
        action_stage = 0;
    }

    void update(bool BL, bool BR, bool FL) {
        using namespace StateCommon;
        // No bajar el elevador ::

        switch (action_stage) {
            // ── Stage 0: retroceder hasta encontrar BR o BL ──────────────────────
            case 0:
                vision.stop();
                vision.clearErrors();
                elevator.ElevatorPosition(kElevatorStop);

                if (BR || BL) {
                    LARC.stop();
                    action_stage = 1;
                    return;
                }

                LARC.backward(kManeuverSpeed);
                return;

            // ── Stage 1: LARC.left hasta encontrar FL ────────────────────────────
            case 1:
                elevator.ElevatorPosition(kElevatorStop);

                if (FL) {
                    LARC.stop();
                    action_stage = 2;
                    return;
                }

                LARC.left(kManeuverSpeed);
                return;

            // ── Stage 2: detenido ─────────────────────────────────────────────────
            case 2:
                elevator.ElevatorPosition(kElevatorStop);
                LARC.stop();
                return;
        }
    }

private:
    int action_stage = 0;
};
