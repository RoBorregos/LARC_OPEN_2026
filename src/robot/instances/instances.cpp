#include "instances.hpp"
#include "pins.h"
#include "constants.h"

// TCA9548A en Wire1 (SDA1=17/SCL1=16), confirmado en hardware con
// vlx_single_test.cpp. Frente (UL/UR) para POOL, atras (LL/LR) para POOLSGOBACK.
// Defined first: the PCA9685 servo board sits behind it too.
TCA9548A i2cMux(0x70, Wire1);

Drive LARC;
Mux74HC4067 mux(Pins::kMuxSig2);
ServoSystem servos(false, &i2cMux); // PCA9685 behind i2cMux, ch Pins::Servos::kTcaChannel
Elevator elevator;

// IR: L1 (FL) y L2 (FR) por GPIO directo, L3 (BL) y L4 (BR) por el mux
// compartido con los QTR (canales libres, ver pins.h).
IRLine ir(Pins::kIrChFL, Pins::kIrChFR, mux,
          Pins::kIrChBLMux, Pins::kIrChBRMux, 0b0000);

Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

QTR qtrFront(Pins::kQtrFrontFirstCh, mux);
QTR qtrRear(Pins::kQtrRearFirstCh, mux, Pins::kQtrRearCount);


VisionLink vision(Serial, servos);

ToF tofLeft(Pins::kToFchFL, i2cMux, ToFType::L1X);
ToF tofRight(Pins::kToFchFR, i2cMux, ToFType::L1X);
ToF tofBackLeft(Pins::kToFchBL, i2cMux, ToFType::L1X);
ToF tofBackRight(Pins::kToFchBR, i2cMux, ToFType::L1X);

OdomMovement odomMove_;