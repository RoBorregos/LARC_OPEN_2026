#include "instances.hpp"
#include "pins.h"
#include "constants.h"

// TCA9548A on Pins::I2cMux::kBus (Wire, SDA=18/SCL=19). Front (UL/UR) for POOL,
// rear (LL/LR) for POOLSGOBACK. The PCA9685 shares this bus directly (no mux channel).
TCA9548A i2cMux(Pins::I2cMux::kAddress, PCA9685::busFromIndex(Pins::I2cMux::kBus));

Drive LARC;
Mux74HC4067 mux(Pins::kMuxSig2);
ServoSystem servos(false, &i2cMux); // mux only used if Pins::Servos::kTcaChannel != 255
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