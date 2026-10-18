/*
 * 03_servo_system_keys_test — stage 3: all five servos via ServoSystem
 *
 *   Drives the real ServoSystem library from single keypresses in the
 *   serial monitor. check every angle in constants.h by hand.
 *
 *     1  toggle upper intake        2  toggle lower intake
 *     3  separator LEFT             4  separator RIGHT
 *     5  separator NEUTRAL
 *     7  open benefit door 1        8  open benefit door 2
 *     9  close + re-arm both doors
 *     0  safe state (everything home, doors shut)
 *     p  print state                h  help
 *     k  print every current angle, paste-ready for constants.h
 *
 *   Angles here come from Constants::ServoConfig and are clamped into each
 *   servo's kCalib limits, so what you see is exactly what the robot will
 *   drive.
 */

#include <Arduino.h>
#include "ServoSystem.hpp"

using namespace Constants::ServoConfig;

ServoSystem servos(true); // timed doors, this test measures the open window

bool upper = false;
bool lower = false;

const char *sepName(ServoSystem::SeparatorPos p)
{
    switch (p)
    {
        case ServoSystem::SeparatorPos::LEFT:  return "LEFT";
        case ServoSystem::SeparatorPos::RIGHT: return "RIGHT";
        default:                               return "NEUTRAL";
    }
}

const char *doorName(ServoSystem::BenefitPhase p)
{
    switch (p)
    {
        case ServoSystem::BenefitPhase::OPEN:       return "OPEN";
        case ServoSystem::BenefitPhase::WAIT_REARM: return "WAIT_REARM";
        default:                                    return "ARMED";
    }
}

// The pulse ServoSystem will actually emit for an angle on this servo.
uint16_t pulseFor(uint8_t servoIndex, uint8_t angleDeg)
{
    const ServoCalib &c = kCalib[servoIndex];
    return (uint16_t)(c.minPulseUs +
                      (uint32_t)(c.maxPulseUs - c.minPulseUs) * angleDeg / 180);
}

void printState()
{
    Serial.printf("  upper=%d(%u deg)  lower=%d(%u deg)  sep=%s(%u deg)\n",
                  upper, servos.lastAngle(INTAKE_UPPER),
                  lower, servos.lastAngle(INTAKE_LOWER),
                  sepName(servos.separatorPos()), servos.lastAngle(SEPARATOR));
    Serial.printf("  door1=%-10s(%u deg)  door2=%-10s(%u deg)\n",
                  doorName(servos.benefitPhase(0)), servos.lastAngle(BENEFIT_1),
                  doorName(servos.benefitPhase(1)), servos.lastAngle(BENEFIT_2));
}

// Every servo's live angle, with the pulse it maps to and the travel limit
// it is clamped into. Handy right before you edit constants.h.
void printAngles()
{
    static const char *kNames[SERVO_COUNT] = {
        "INTAKE_UPPER", "INTAKE_LOWER", "SEPARATOR", "BENEFIT_1", "BENEFIT_2"
    };

    Serial.println();
    Serial.println("  servo          ch   angle    pulse     limits (kCalib)");
    Serial.println("  ------------------------------------------------------");
    for (uint8_t i = 0; i < SERVO_COUNT; ++i)
    {
        const uint8_t a = servos.lastAngle(i);
        if (a == 255)
        {
            Serial.printf("  %-13s %2u   (never written)\n", kNames[i], kCalib[i].channel);
            continue;
        }
        Serial.printf("  %-13s %2u   %3u deg  %4u us   %u..%u deg  %u..%u us\n",
                      kNames[i], kCalib[i].channel, a, pulseFor(i, a),
                      kCalib[i].minAngleDeg, kCalib[i].maxAngleDeg,
                      kCalib[i].minPulseUs, kCalib[i].maxPulseUs);
    }
    Serial.println();
    Serial.println("  // paste-ready — rename each to the position it represents:");
    for (uint8_t i = 0; i < SERVO_COUNT; ++i)
    {
        const uint8_t a = servos.lastAngle(i);
        if (a == 255) continue;
        Serial.printf("  static constexpr uint8_t k%s???? = %u;\n", kNames[i], a);
    }
    Serial.println();
}

void printHelp()
{
    Serial.println("[03_servo_system_keys_test] ServoSystem by hand");
    Serial.println("  1/2 intake upper/lower   3/4/5 separator L/R/neutral");
    Serial.println("  7/8 open door 1/2        9 close+re-arm both");
    Serial.println("  0 safe state             p print state   h help");
    Serial.println("  k print all angles + paste-ready constants");
    Serial.printf("  benefit open window = %lu ms\n", (unsigned long)kBenefitOpenMs);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    servos.begin(); // also closes and re-arms both benefit doors

    printHelp();
    printState();
}

void loop()
{
    servos.update();

    if (!Serial.available())
        return;

    bool changed = true;
    switch (Serial.read())
    {
        case '1': upper = !upper; servos.setIntakeUpper(upper); break;
        case '2': lower = !lower; servos.setIntakeLower(lower); break;
        case '3': servos.setSeparator(ServoSystem::SeparatorPos::LEFT);    break;
        case '4': servos.setSeparator(ServoSystem::SeparatorPos::RIGHT);   break;
        case '5': servos.setSeparator(ServoSystem::SeparatorPos::NEUTRAL); break;
        case '7': servos.setBenefit(0, true);  break;
        case '8': servos.setBenefit(1, true);  break;
        case '9': servos.closeBenefits();      break;
        case '0': upper = lower = false; servos.safeState(); break;
        case 'p': break;
        case 'k': printAngles(); changed = false; break;
        case 'h': printHelp();   changed = false; break;
        default:  changed = false; break;
    }

    if (changed)
        printState();
}
