/** 
 * Author: Ximena Patricia Garcia Magdaleno
 * 
 * 03/03/2025 
 * 
 * Elevator.cpp file from Elevator class
 * 
 * Linear Elevator Actuator
 *
 * Bidirectional elevator mechanism driven by a DC motor.
 * Direction is controlled through an H-Bridge using INA1 and INA2,
 * while motion speed is adjusted through PWM control.
 *
 * States:
 * 0 -> Stop
 * 1 -> Move Up
 * 2 -> Move Down
 * 
**/
#include "Elevator.hpp"

Elevator::Elevator() : state(0), pin1(Pins::kElevator[0]), pin2(Pins::kElevator[1]), pwm(Pins::kPwmPin[4])
{

}

void Elevator::begin()
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwm, OUTPUT);
}


void Elevator::ElevatorPosition(int state_)
{
    state = state_;
    moveElevator(state);
}
    

void Elevator::moveElevator(int direction)
{
    if (direction == 1){ //UP
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, 180);
        //delay(7000); // It can be omitted for now.
    }
    else if(direction == 2){ // DOWN
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, 180);   // Velocity of 0 a 255
        //delay(7000); 
    }
    else{
        analogWrite(pwm, 0); //elevator stop

    }


}

/*
#include <Arduino.h>
#include "pins.h"

const uint8_t p1  = Pins::kElevatorINA1;
const uint8_t p2  = Pins::kElevatorINA2;
const uint8_t pwm = Pins::kElevatorPWM;

void setup()
{
    pinMode(p1, OUTPUT);
    pinMode(p2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void loop()
{
    // Stop
    analogWrite(pwm, 0);
    delay(2000);

    // Bajar
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwm, 180);   // velocidad de 0 a 255
    delay(7000);

    // Stop
    analogWrite(pwm, 0);
    delay(2000);

    // Subir
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwm, 180);
    delay(7000);

    // Stop
    analogWrite(pwm, 0);
    delay(2000);
    
}
*/
