/*
 * 01_servo_direct_pin_test — stage 1: is the servo itself alive?
 *   Sweeps ONE servo between two angles, once per second, forever, using
 *   the plain Arduino <Servo.h> library on a Teensy pin. 
 */

#include <Arduino.h>
#include <Servo.h>

constexpr uint8_t  kServoPin  = 2;
constexpr uint8_t  kSafeAngle = 50;   // known-safe resting angle
constexpr uint8_t  kTestAngle = 85; // angle to visit each cycle
constexpr uint32_t kDwellMs   = 1000; // hold time at each position

Servo servo;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    servo.attach(kServoPin);
    servo.write(kSafeAngle);

    Serial.println("[01_servo_direct_pin_test] bare servo on a Teensy pin");
    Serial.printf("  pin=%u  safe=%u deg  test=%u deg  dwell=%lu ms\n",
                  kServoPin, kSafeAngle, kTestAngle, (unsigned long)kDwellMs);
}

void loop()
{
    Serial.printf("-> test angle %u\n", kTestAngle);
    servo.write(kTestAngle);
    delay(kDwellMs);

    Serial.printf("-> safe angle %u\n", kSafeAngle);
    servo.write(kSafeAngle);
    delay(kDwellMs);
}
