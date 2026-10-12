/*

* Odometry Test Using Wheel Encoders
*
* This code tests the robot's odometry using only the wheel encoders,
* without using the BNO sensor for yaw correction.
*
* At startup, the Drive subsystem is initialized, yaw holding is disabled,
* and the odometry values are reset to zero.
*
* The robot then moves forward at a speed of 0.45 and continuously updates
* its estimated position. Once the absolute X displacement reaches 0.5 meters,
* the robot applies the brakes.
*
* Every 100 milliseconds, the program prints:
* * The distance traveled by each motor: M1, M2, M3, and M4
* * The estimated X position
* * The estimated Y position
*
* The serial output can be used to verify the encoder measurements,
* wheel calibration, and odometry calculations.
*/

#include <Arduino.h>
#include "Subsystem/Drive/Drive.hpp"

Drive drive;

static bool    started  = false;
static uint32_t lastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    drive.begin();
    drive.holdYaw(false);   // Without BNO, using encoders only
    drive.resetOdometry();

    Serial.println("=== TEST ODOMETRIA ===");
    Serial.println("Formato: M1 M2 M3 M4 | OdoX OdoY");
}

void loop() {
    drive.update();

    // Starts only once
    if (!started) {
        started = true;
        drive.forward(0.45f);
    }

    // Stops at 0.5m
    if (fabsf(drive.getOdoX()) >= 0.5f) {
        drive.brake();
    }

    // Print at 10 Hz
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print(drive.getM1Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM2Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM3Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM4Meters(), 3); Serial.print("\t");
        Serial.print(drive.getOdoX(), 3);     Serial.print("\t");
        Serial.println(drive.getOdoY(), 3);
    }
}