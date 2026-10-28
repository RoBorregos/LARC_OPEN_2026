/*
   teensy_separator_test.cpp — the Teensy half of separator_vision.py
   PAIRS WITH   vision_orin/separator/separator_vision.py
   TUNE WITH    vision_orin/separator/separator_debug.py
                (that one can drive this sketch straight from a Mac with
                 --teensy, no Orin and no dispatcher in the loop)
  
   WHAT IT DOES
     Holds the BEANS phase and reports ONLY the separator. Every time the
     commanded side changes you get one line with the new side, the angle
     the servo went to, and a running count per side — so a camera that
     flickers between warm and cool is obvious from the counters alone.
  
   HOW TO RUN — with Orin
     1. Do NOT open a serial monitor — the dispatcher owns that port.
     3. journalctl -u larc-dispatcher -f
  
   HOW TO RUN (no Orin)
     1. Same upload.
     2. sudo systemctl stop larc-dispatcher   (if the Orin is also plugged in)
     3. python3 vision_orin/separator/separator_debug.py --teensy
        That tool streams the BEANS frames itself and prints these lines
        back into your terminal..
 */

#include <Arduino.h>
#include "ServoSystem.hpp"
#include "VisionLink.hpp"

using namespace Constants::ServoConfig;

constexpr uint32_t kSettleMs       = 750;
constexpr uint32_t kRequestRetryMs = 1000;
constexpr uint32_t kHeartbeatMs    = 2000;

ServoSystem servos;
VisionLink  vision(Serial, servos);

ServoSystem::SeparatorPos pSep = ServoSystem::SeparatorPos::NEUTRAL;
bool     pLink = false;
uint32_t nLeft = 0, nRight = 0, nNeutral = 0;
uint32_t lastRequest = 0, lastBeat = 0;
bool     started = false;

const char *sepName(ServoSystem::SeparatorPos p)
{
    switch (p)
    {
        case ServoSystem::SeparatorPos::LEFT:  return "LEFT";
        case ServoSystem::SeparatorPos::RIGHT: return "RIGHT";
        default:                               return "NEUTRAL";
    }
}

void setup()
{
    Serial.begin(Constants::VisionConfig::kSerialBaud);
    servos.begin();
    vision.begin();

    Serial.println();
    Serial.println("[sep] teensy_separator_test — BEANS phase, separator only");
    Serial.printf("[sep] channel %u   travel %u..%u deg   "
                  "neutral=%u left=%u right=%u\n",
                  kCalib[SEPARATOR].channel, kCalib[SEPARATOR].minAngleDeg,
                  kCalib[SEPARATOR].maxAngleDeg,
                  kSeparatorNeutral, kSeparatorLeft, kSeparatorRight);
    Serial.println("[sep] warm ball -> LEFT (mature)   cool ball -> RIGHT (immature)");
}

void loop()
{
    vision.update();
    const uint32_t now = millis();

    if (now >= kSettleMs && vision.phase() != VisionProto::Phase::BEANS &&
        now - lastRequest >= kRequestRetryMs)
    {
        if (!started) { Serial.println("[sep] requesting BEANS"); started = true; }
        lastRequest = now;
        vision.resetGuards();
        vision.startBeans();
    }

    if (servos.separatorPos() != pSep)
    {
        pSep = servos.separatorPos();
        if      (pSep == ServoSystem::SeparatorPos::LEFT)  nLeft++;
        else if (pSep == ServoSystem::SeparatorPos::RIGHT) nRight++;
        else                                               nNeutral++;

        Serial.printf("[sep] %-8s %u deg   (L=%lu R=%lu N=%lu)\n",
                      sepName(pSep), servos.lastAngle(SEPARATOR),
                      (unsigned long)nLeft, (unsigned long)nRight,
                      (unsigned long)nNeutral);
    }
    if (vision.lastSeparatorInvalid())
        Serial.println("[sep] ! reserved separator code received -> forced NEUTRAL");

    if (vision.isLinkUp() != pLink)
    {
        pLink = vision.isLinkUp();
        Serial.printf("[sep] link %s\n", pLink ? "UP" : "LOST -> SAFE");
    }

    if (now - lastBeat >= kHeartbeatMs)
    {
        lastBeat = now;
        Serial.printf("[sep] phase=%s now=%s(%u deg)  L=%lu R=%lu N=%lu  "
                      "acc=%lu rej=%lu\n",
                      vision.inBeansPhase() ? "BEANS" : "not-BEANS",
                      sepName(servos.separatorPos()), servos.lastAngle(SEPARATOR),
                      (unsigned long)nLeft, (unsigned long)nRight,
                      (unsigned long)nNeutral,
                      (unsigned long)vision.stats().accepted,
                      (unsigned long)vision.stats().rejected);
    }
}
