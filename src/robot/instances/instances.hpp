#ifndef ROBOT_INSTANCES_H
#define ROBOT_INSTANCES_H

#include <Arduino.h>
#include "pins.h"
#include "constants.h"
//Sensors
#include "IR/IR.hpp"
#include "mux.h"
#include "qtr.hpp"
#include "ultrasonic/ultrasonic.hpp"
#include "ServoSystem.hpp"
#include "tof/tof.hpp"
//Elevator
#include "Elevator.hpp"
//Drive
#include "subsystem/Drive/Drive.hpp"
#include "PIDController.hpp"
// Vision
#include "Vision.hpp"
// VLX
#include "TCA9548A/TCA9548A.h"
// Odometry
#include "testOdometry.hpp"


extern Drive LARC;
extern Mux74HC4067 mux;
extern ServoSystem servos;
extern Elevator elevator;

extern IRLine ir;
extern Ultrasonic us1;
extern Ultrasonic us2;
extern QTR qtrFront;
extern QTR qtrRear;


extern PIDController linePID;

extern Vision vision;

extern ToF tofLeft;
extern ToF tofRight;

extern TCA9548A i2cMux;

extern OdomMovement odomMove_;

#endif