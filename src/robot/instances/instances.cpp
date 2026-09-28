#include "instances.hpp"
#include "pins.h"

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
QTR qtrRear(Pins::kQtrRearFirstCh, mux);

PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f);

VisionLink vision(Serial, servos);

// Wire1 (SDA 17 / SCL 16): ToFs and the PCA9685 servo driver sit behind it.
TCA9548A i2cMux(Pins::I2cMux::kAddress, PCA9685::busFromIndex(Pins::I2cMux::kBus));
ToF tofLeft(Pins::kToFchFL, i2cMux, ToFType::L1X);
ToF tofRight(Pins::kToFchFR, i2cMux, ToFType::L1X);
ToF tofBackLeft(Pins::kToFchBL, i2cMux, ToFType::L1X);
ToF tofBackRight(Pins::kToFchBR, i2cMux, ToFType::L1X);

OdomMovement odomMove_;