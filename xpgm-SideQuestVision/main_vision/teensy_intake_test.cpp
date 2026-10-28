/*
   teensy_intake_test.cpp — the Teensy half of orin_vision.py
   PAIRS WITH   vision_orin/main_vision/orin_vision.py  (the ZED intake)
   TUNE WITH    vision_orin/main_vision/orin_vision_debug.py
  
    WHAT IT DOES
    Holds the BEANS phase and reports ONLY the intake. Every time
    orin_vision.py changes a bean bit you get one line with the bit, the
    servo it moved and the angle it went to — so you can see the whole
    chain (camera -> VISION:XX -> payload bit -> servo angle) in one place.
    The separator moves too if separator_vision.py is running; this sketch
    just does not talk about it.
  
    HOW TO RUN
    1. Do NOT open a serial monitor — the dispatcher owns that port.
    3. On the Orin:  journalctl -u larc-dispatcher -f

 */

#include <Arduino.h>
#include "ServoSystem.hpp"
#include "VisionLink.hpp"

using namespace Constants::ServoConfig;

constexpr uint32_t kSettleMs       = 750;   // let the servos reach safe first
constexpr uint32_t kRequestRetryMs = 1000;  // re-ask until the phase sticks
constexpr uint32_t kHeartbeatMs    = 2000;  // slow: changes are the signal here

ServoSystem servos;
VisionLink  vision(Serial, servos);

bool     pUp = false, pLo = false, pLink = false;
uint32_t upHits = 0, loHits = 0;
uint32_t lastRequest = 0, lastBeat = 0;
bool     started = false;

void setup()
{
    Serial.begin(Constants::VisionConfig::kSerialBaud);
    servos.begin();
    vision.begin();

    Serial.println();
    Serial.println("[intake] teensy_intake_test — BEANS phase, intake only");
    Serial.printf("[intake] upper ch%u %u..%u deg   lower ch%u %u..%u deg\n",
                  kCalib[INTAKE_UPPER].channel, kCalib[INTAKE_UPPER].minAngleDeg,
                  kCalib[INTAKE_UPPER].maxAngleDeg,
                  kCalib[INTAKE_LOWER].channel, kCalib[INTAKE_LOWER].minAngleDeg,
                  kCalib[INTAKE_LOWER].maxAngleDeg);
}

void loop()
{
    vision.update();
    const uint32_t now = millis();

    // Keep asking until we are actually in BEANS. One lost byte would
    // otherwise leave this sitting in IDLE looking perfectly healthy.
    if (now >= kSettleMs && vision.phase() != VisionProto::Phase::BEANS &&
        now - lastRequest >= kRequestRetryMs)
    {
        if (!started) { Serial.println("[intake] requesting BEANS"); started = true; }
        lastRequest = now;
        vision.resetGuards();
        vision.startBeans();
    }

    if (servos.intakeUpperDeployed() != pUp)
    {
        pUp = servos.intakeUpperDeployed();
        if (pUp) upHits++;
        Serial.printf("[intake] UPPER  %-3s  %u deg   (hits: up=%lu lo=%lu)\n",
                      pUp ? "ON" : "off", servos.lastAngle(INTAKE_UPPER),
                      (unsigned long)upHits, (unsigned long)loHits);
    }

    if (servos.intakeLowerDeployed() != pLo)
    {
        pLo = servos.intakeLowerDeployed();
        if (pLo) loHits++;
        Serial.printf("[intake] LOWER  %-3s  %u deg   (hits: up=%lu lo=%lu)\n",
                      pLo ? "ON" : "off", servos.lastAngle(INTAKE_LOWER),
                      (unsigned long)upHits, (unsigned long)loHits);
    }

    if (vision.isLinkUp() != pLink)
    {
        pLink = vision.isLinkUp();
        Serial.printf("[intake] link %s\n", pLink ? "UP" : "LOST -> SAFE");
    }

    if (now - lastBeat >= kHeartbeatMs)
    {
        lastBeat = now;
        Serial.printf("[intake] phase=%s up=%u deg lo=%u deg  hits up=%lu lo=%lu  "
                      "acc=%lu rej=%lu\n",
                      vision.inBeansPhase() ? "BEANS" : "not-BEANS",
                      servos.lastAngle(INTAKE_UPPER), servos.lastAngle(INTAKE_LOWER),
                      (unsigned long)upHits, (unsigned long)loHits,
                      (unsigned long)vision.stats().accepted,
                      (unsigned long)vision.stats().rejected);
    }
}
