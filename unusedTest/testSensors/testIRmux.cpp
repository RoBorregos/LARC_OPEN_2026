#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include <math.h>

#include "IR_mux/IR_mux.hpp"
#include "mux.h"
#include "subsystem/Drive/Drive.hpp"

Drive Robot;
Mux74HC4067 mux;

static constexpr float velocity = Constants::PID::kcurrentVelocity; 

const uint8_t irChannels[IR_mux::N] = { 13, 12, 11, 10 };

// invertedMask = 0b0010 -> invierte el sensor índice 1 (normalmente FR)
IR_mux ir(mux, irChannels, 0b000);

enum RobotState
{
    FORWARD,
    BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
    STOP_STATE
};

RobotState currentState = FORWARD;

void updateState(bool FL, bool FR, bool BL, bool BR)
{
    const bool backDetected  = (BL || BR); 
    const bool leftDetected  = (FL || BL); 
    const bool rightDetected = (FR || BR); 
    const bool frontDetected = (FL || FR); 

    if (backDetected)
        currentState = FORWARD;
    else if (leftDetected)
        currentState = MOVE_RIGHT;
    else if (rightDetected)
        currentState = MOVE_LEFT;
    else if (frontDetected)
        currentState = BACKWARD;   
    else
        currentState = FORWARD;
}

void executeState()
{
    switch (currentState)
    {
    case MOVE_LEFT:   Robot.left(velocity); break;    //Serial.println("left");   break;
    case MOVE_RIGHT:  Robot.right(velocity);  break;  //Serial.println("right");    break;
    case BACKWARD:    Robot.backward(velocity); break; //Serial.println("backward"); break;
    case STOP_STATE:  Robot.brake();     break;     //Serial.println("stop");         break;
    case FORWARD:
    default:          Robot.forward(velocity);  break;//Serial.println("FORWARD");  break;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000); 

    Robot.begin();
    Robot.holdYaw(true);
    Robot.setTargetYaw(Robot.getYaw());

    ir.begin();
    Serial.println(F("IR_mux listo."));
}

void loop()
{
    ir.update();
    ir.debugPrint();
    bool fl = ir.getState(IR_mux::FL);
    bool fr = ir.getState(IR_mux::FR);
    bool bl = ir.getState(IR_mux::BL);
    bool br = ir.getState(IR_mux::BR);

    updateState(fl, fr, bl, br);
    executeState();

    delay(100);
}