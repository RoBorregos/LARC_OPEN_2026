#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "qtr.hpp"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"
#include "constants.h"

Mux74HC4067 mux;
QTR qtrFront(0, mux);
Drive LARC;
PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f); 

static constexpr float kBaseSpeed = 0.45f;// strafing speed

void setup() {
  Serial.begin(115200);
  delay(500);

  mux.begin();
  qtrFront.begin();
  qtrFront.useDefaultCalibration();

  LARC.begin();
  LARC.holdYaw(true);
  LARC.setTargetYaw(LARC.getYaw());
}

void loop() {
  qtrFront.update();

  int linePos = qtrFront.getBinaryPosition();
  bool  onLine   = qtrFront.onLine();
  float lineErr  = linePos - Constants::LineFollower::kSetpoint;
  float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
  float vx       = -lineCorr;

  Serial.print("pos:");     Serial.print(linePos);
  Serial.print("\tonLine:"); Serial.print(onLine);
  Serial.print("\terr:");    Serial.print(lineErr);
  Serial.print("\tvx:");     Serial.println(vx);
  qtrFront.debugPrint();

  LARC.setTranslation(vx, kBaseSpeed); //Change sign of kBaseSpeed to strafe LEFT (+) or RIGHT (-)
  LARC.update();

  delay(10);
}