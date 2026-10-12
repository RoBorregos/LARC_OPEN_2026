#include <Arduino.h>
#include <Wire.h>
#include "subsystem/Drive/Drive.hpp"

static constexpr float kSideMeters = 0.80f;
static constexpr float kTargetRPM  = 60.0f;
static constexpr float kStopTolM   = 0.03f;
static constexpr uint32_t kPausems = 600;

enum class Step : uint8_t { FORWARD, LEFT, PAUSE, DONE };

Drive    drive;
Step     step       = Step::FORWARD;
uint32_t pauseStart = 0;
uint8_t  stepCount  = 0;

void startStep(Step s) {
    step = s;
    drive.resetOdometry();
    drive.holdYaw(true);

    switch (s) {
        case Step::FORWARD:
            Serial.println(">>> FORWARD");
            drive.setRPMs(+kTargetRPM, +kTargetRPM, +kTargetRPM, +kTargetRPM);
            break;
        case Step::LEFT:
            Serial.println(">>> LEFT");
            drive.setRPMs(-kTargetRPM, +kTargetRPM, +kTargetRPM, -kTargetRPM);
            break;
        default:
            break;
    }
}

void setup() {
    drive.begin();
    delay(2000);
    Serial.println("=== CUADRADO CON CLASES ===");
    startStep(Step::FORWARD);
}

void loop() {
    drive.update();

    const uint32_t now = millis();

    switch (step) {
        case Step::FORWARD:
        case Step::LEFT: {
            const float dist = sqrtf(
                drive.getOdoX() * drive.getOdoX() +
                drive.getOdoY() * drive.getOdoY());

            if (dist >= kSideMeters - kStopTolM) {
                drive.stop();
                Serial.print(">>> Llegue! dist=");
                Serial.println(dist, 3);
                pauseStart = now;
                step = Step::PAUSE;
            }

            static uint32_t lastPrint = 0;
            if (now - lastPrint >= 200) {
                lastPrint = now;
                Serial.print("x:");     Serial.print(drive.getOdoX(), 3);
                Serial.print(" y:");    Serial.print(drive.getOdoY(), 3);
                Serial.print(" dist:"); Serial.print(dist, 3);
                Serial.print(" yaw:");  Serial.print(drive.getYaw() * 180.0f / M_PI, 1);
                Serial.print(" step:"); Serial.println(step == Step::FORWARD ? "FWD" : "LFT");
            }
            break;
        }

        case Step::PAUSE:
            if (now - pauseStart >= kPausems) {
                stepCount++;
                if (stepCount == 1) {
                    startStep(Step::LEFT);
                } else {
                    step = Step::DONE;
                    Serial.println("=== SECUENCIA COMPLETA ===");
                }
            }
            break;

        case Step::DONE:
            break;
    }
}