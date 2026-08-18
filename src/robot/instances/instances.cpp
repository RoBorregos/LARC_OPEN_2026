#include "instances.hpp"
#include "pins.h"

Drive LARC;
Mux74HC4067 mux(Pins::kMuxSig);
ServoSystem servos;
Elevator elevator;

// IR directos
const uint8_t irPins[IRLine::N] = {
    Pins::kIrChFL,
    Pins::kIrChFR,
    Pins::kIrChBL,
    Pins::kIrChBR
};

IRLine ir(irPins, 0b0000);

Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

QTR qtrFront(Pins::kQtrFrontFirstCh, mux);
QTR qtrRear(Pins::kQtrRearFirstCh, mux);

PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f);

Vision vision(Serial);

TCA9548A i2cMux;
ToF tofLeft(Pins::kToFchFL, i2cMux, ToFType::L1X);
ToF tofRight(Pins::kToFchFR, i2cMux, ToFType::L1X);

OdomMovement odomMove_;