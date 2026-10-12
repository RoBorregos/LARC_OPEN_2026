// Ultrasonics + IR (avoid pools complete task) test

#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include <math.h>

#include "IR_mux/IR_mux.hpp"
#include "mux.h"
#include "subsystem/Drive/Drive.hpp"
#include "ultrasonic/ultrasonic.hpp"

Drive LARC;
Mux74HC4067 mux;

// Ultrasonics
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

static constexpr float velocity = 0.45f;
static constexpr float kObstacleDistanceCm = 15.0f;
static constexpr uint32_t kClearDelayMs = 300;

// IR
const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
IR_mux ir(mux, irChannels, 0b000);

enum class State
{
    FORWARD,
    SEARCH_LEFT,
    SEARCH_RIGHT,
    STOP_STATE
};

State currentState = State::FORWARD;

static uint32_t clearStartMs = 0;

static bool obstacleHandled = false;

void setState(State newState)
{
    currentState = newState;
    clearStartMs = 0;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    LARC.begin();
    LARC.holdYaw(true);
    LARC.setTargetYaw(LARC.getYaw());

    ir.begin();

    us1.begin();
    us2.begin();

    currentState = State::FORWARD;

    Serial.println(F("Ready to start testUSandIR.cpp file (Avoid obstacles complete task TEST)"));
}

void loop()
{
    LARC.update();
    ir.update();
    us1.update();
    us2.update();

    ir.debugPrint();

    // IRs
    bool FL = ir.getState(IR_mux::FL);
    bool FR = ir.getState(IR_mux::FR);
    bool BL = ir.getState(IR_mux::BL);
    bool BR = ir.getState(IR_mux::BR);

    const bool leftDetected  = (FL || BL);
    const bool rightDetected = (FR || BR);
    const bool frontDetected = (FL && FR);

    // Ultrasonicos
    float d1 = us1.getdistance();
    float d2 = us2.getdistance();

    bool obstacle =
        (us1.isValid() && d1 < kObstacleDistanceCm) ||
        (us2.isValid() && d2 < kObstacleDistanceCm);

    const uint32_t now = millis();

    switch (currentState)
    {
    case State::FORWARD:
        if (obstacleHandled && frontDetected)
        {
            setState(State::STOP_STATE);
        }
        else if (obstacle)
        {
            setState(State::SEARCH_LEFT);
        }
        else
        {
            LARC.forward(velocity);
        }
        break;

    case State::SEARCH_LEFT:
        LARC.left(velocity);

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if (now - clearStartMs >= kClearDelayMs)
            {
                obstacleHandled = true;
                setState(State::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }

       
        if (leftDetected)
        {
            setState(State::SEARCH_RIGHT);
        }
        break;

    case State::SEARCH_RIGHT:
        LARC.right(velocity);

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if (now - clearStartMs >= kClearDelayMs)
            {
                obstacleHandled = true;
                setState(State::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
        break;

    case State::STOP_STATE:
    default:
        LARC.brake();
        break;
    }
}