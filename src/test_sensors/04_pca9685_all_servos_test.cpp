/*
  04_pca9685_all_servos_test — the five robot servos through the PCA9685
 
    Same idea as 02_pca9685_channel_test, but for all five servos at once.
    Drives the PCA9685 directly with lib/PCA9685, deliberately bypassing
    ServoSystem: NO CLAMPING!!!
 
  calibration procedure (per servo)
    1.  1-5    pick the servo.
    2.  ] / [  walk toward one mechanical stop, one degree at a time.
               Stop the moment it touches WITHOUT straining or buzzing.
    3.  m      mark that angle as this servo's minAngleDeg.
    4.  ] / [  walk to the other stop the same way.
    5.  M      mark that angle as maxAngleDeg.
    6.  k      print the paste-ready kCalib row -> constants.h.
 
    keys
    instant (no Enter needed)
      1-5  select servo     (1 intake upper, 2 intake lower, 3 separator,
                             4 benefit 1, 5 benefit 2)
      s  rest angle (home / neutral / closed, from constants.h)
      t  work angle (deploy / left / open, from constants.h)
      c  90 deg
      +  +25 us            -  -25 us
      ]  +1 deg            [  -1 deg
      m  mark MIN angle    M  mark MAX angle     z  clear marks
      k  print paste-ready constants
      p  print this servo  P  print all servos
      d  detach this servo D  detach all servos  h  help
 
    typed, then Enter
      a<deg>   go to an angle       e.g.  a90    a132.5
      u<us>    go to a pulse width  e.g.  u1500
 */

#include <Arduino.h>
#include "PCA9685.hpp"
#include "TCA9548A/TCA9548A.h"
#include "pins.h"
#include "constants.h"

using namespace Constants::ServoConfig;

// ── Test configuration (edit here only) ─────────────────────────────────
constexpr uint8_t  kPcaAddress  = Pins::Servos::kPcaI2cAddress; // board + bus from pins.h
constexpr uint32_t kOscHz       = kPcaOscillatorHz;
constexpr float    kServoFreqHz = kServoPwmFreqHz;

constexpr uint16_t kNudgeUs  = 25;    // step for + / -
constexpr float    kNudgeDeg = 1.0f;  // step for ] / [
// ────────────────────────────────────────────────────────────────────────

struct Slot
{
    const char *name;     // as printed
    const char *chName;   // Pins::Servos constant, for the paste row
    uint8_t     restDeg;  // s
    uint8_t     workDeg;  // t
    uint16_t    us;       // last pulse written
    bool        attached; // pulsing right now
    int16_t     markMin;  // -1 = not marked
    int16_t     markMax;
};

// Order matches Constants::ServoConfig::ServoIndex, so kCalib[i] is slot i.
static Slot slots[SERVO_COUNT] = {
    { "intake upper", "kIntakeUpperCh", kIntakeUpperHome,  kIntakeUpperDeploy, 1500, false, -1, -1 },
    { "intake lower", "kIntakeLowerCh", kIntakeLowerHome,  kIntakeLowerDeploy, 1500, false, -1, -1 },
    { "separator",    "kSeparatorCh",   kSeparatorNeutral, kSeparatorLeft,     1500, false, -1, -1 },
    { "benefit 1",    "kBenefit1Ch",    kBenefit1Closed,   kBenefit1Open,      1500, false, -1, -1 },
    { "benefit 2",    "kBenefit2Ch",    kBenefit2Closed,   kBenefit2Open,      1500, false, -1, -1 },
};

// static + own name: instances.cpp (always linked) already defines i2cMux
static TCA9548A benchMux(Pins::I2cMux::kAddress, PCA9685::busFromIndex(Pins::I2cMux::kBus));
static PCA9685  pwm(kPcaAddress, PCA9685::busFromIndex(Pins::Servos::kI2cBus), Pins::Servos::kOePin);

static uint8_t sel = INTAKE_UPPER;

static char    lineBuf[16];
static uint8_t lineLen = 0;

// ── Pulse <-> angle, per servo ──────────────────────────────────────────
float usToDeg(uint8_t i, uint16_t us)
{
    const ServoCalib &c = kCalib[i];
    return (float)((int32_t)us - (int32_t)c.minPulseUs) * 180.0f /
           (float)((int32_t)c.maxPulseUs - (int32_t)c.minPulseUs);
}

uint16_t degToUs(uint8_t i, float deg)
{
    const ServoCalib &c = kCalib[i];
    return (uint16_t)lroundf((float)c.minPulseUs +
        deg * (float)((int32_t)c.maxPulseUs - (int32_t)c.minPulseUs) / 180.0f);
}

// Teensy printf and %f do not always agree. Print tenths by hand instead.
void printDeg(float deg)
{
    int32_t tenths = lroundf(deg * 10.0f);
    bool neg = tenths < 0;
    if (neg) tenths = -tenths;
    Serial.printf("%s%ld.%ld", neg ? "-" : "", (long)(tenths / 10), (long)(tenths % 10));
}

// ── Movement ────────────────────────────────────────────────────────────
void showServo(uint8_t i, const char *tag)
{
    const Slot &s = slots[i];
    const float deg = usToDeg(i, s.us);

    Serial.printf("  %c%u %-12s ch %-2u  ", i == sel ? '>' : ' ', i + 1, s.name, kCalib[i].channel);
    if (s.attached)
    {
        Serial.printf("%4u us  ", s.us);
        printDeg(deg);
        Serial.print(" deg");
    }
    else
    {
        Serial.print("  silent (no pulses)");
    }

    if (s.markMin >= 0 || s.markMax >= 0)
    {
        Serial.print("   marks[");
        if (s.markMin >= 0) Serial.printf("min %d", s.markMin); else Serial.print("min --");
        Serial.print(" | ");
        if (s.markMax >= 0) Serial.printf("max %d", s.markMax); else Serial.print("max --");
        Serial.print("]");

        const int16_t lo = (s.markMin >= 0) ? s.markMin : 0;
        const int16_t hi = (s.markMax >= 0) ? s.markMax : 180;
        if (s.attached && (deg < (float)lo - 0.5f || deg > (float)hi + 0.5f))
            Serial.print("  <-- OUTSIDE marked band");
    }

    if (tag && *tag) Serial.printf("   %s", tag);
    Serial.println();
}

void moveToUs(const char *tag, int32_t us)
{
    const ServoCalib &c = kCalib[sel];
    if (us < (int32_t)c.minPulseUs) us = c.minPulseUs;
    if (us > (int32_t)c.maxPulseUs) us = c.maxPulseUs;

    Slot &s = slots[sel];
    s.us       = (uint16_t)us;
    s.attached = true;
    pwm.writeMicroseconds(c.channel, s.us);
    showServo(sel, tag);
}

void moveToDeg(const char *tag, float deg)
{
    if (deg <   0.0f) deg =   0.0f;
    if (deg > 180.0f) deg = 180.0f;
    moveToUs(tag, degToUs(sel, deg));
}

void detach(uint8_t i)
{
    pwm.off(kCalib[i].channel); // no pulses at all — servo goes limp
    slots[i].attached = false;
}

void selectServo(uint8_t i)
{
    sel = i;
    showServo(sel, slots[sel].attached ? "selected" : "selected — press s / t / a<deg> to drive it");
}

void printAll()
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        showServo(i, "");
}

// ── Paste-ready output ──────────────────────────────────────────────────
void printPaste()
{
    const Slot &s = slots[sel];
    const ServoCalib &c = kCalib[sel];

    Serial.println();
    Serial.printf("// ─── %s: paste into Constants::ServoConfig::kCalib[] (include/constants.h) ───\n", s.name);

    if (s.markMin < 0 || s.markMax < 0)
    {
        Serial.println("//  ! stops not marked yet — walk to each mechanical stop and press");
        Serial.println("//    m (min) and M (max). Current kCalib limits shown for the missing ones.");
    }

    const int lo = (s.markMin >= 0) ? s.markMin : c.minAngleDeg;
    const int hi = (s.markMax >= 0) ? s.markMax : c.maxAngleDeg;
    Serial.printf("{ Pins::Servos::%s, %u, %u, %d, %d },\n",
                  s.chName, c.minPulseUs, c.maxPulseUs, lo <= hi ? lo : hi, lo <= hi ? hi : lo);

    if (s.attached)
    {
        Serial.printf("// current position: %ld deg (", (long)lroundf(usToDeg(sel, s.us)));
        printDeg(usToDeg(sel, s.us));
        Serial.printf(" deg = %u us)\n", s.us);
    }
    Serial.println();
}

void printHelp()
{
    Serial.println();
    Serial.println("[04_pca9685_all_servos_test] five servos, in degrees, no ServoSystem");
    Serial.printf("  addr=0x%02X  freq=%.0f Hz  (angles use each servo's kCalib pulse map)\n",
                  kPcaAddress, kServoFreqHz);
    Serial.println("  instant keys:");
    Serial.println("    1-5     select servo (the one you leave keeps holding)");
    Serial.println("    s t c   rest / work angle (constants.h) / 90 deg");
    Serial.println("    + -     pulse +/- 25 us        ] [   angle +/- 1 deg");
    Serial.println("    m M     mark MIN / MAX angle   z     clear marks");
    Serial.println("    k       print paste-ready kCalib row");
    Serial.println("    p P     print this / all servos");
    Serial.println("    d D     detach this / all servos      h  help");
    Serial.println("  typed, then Enter:");
    Serial.println("    a<deg>  go to angle    u<us>  go to pulse");
    printAll();
}

// ── Input ───────────────────────────────────────────────────────────────
void runInstant(char c)
{
    Slot &s = slots[sel];

    switch (c)
    {
        case '1': case '2': case '3': case '4': case '5':
            selectServo((uint8_t)(c - '1'));
            break;

        case 's': moveToDeg("rest", s.restDeg); break;
        case 't': moveToDeg("work", s.workDeg); break;
        case 'c': moveToDeg("90",   90.0f);     break;

        // From silent, nudges start from the rest angle, not a random pulse.
        case '+': moveToUs("nudge", (s.attached ? (int32_t)s.us : degToUs(sel, s.restDeg)) + kNudgeUs); break;
        case '-': moveToUs("nudge", (s.attached ? (int32_t)s.us : degToUs(sel, s.restDeg)) - kNudgeUs); break;
        case ']': moveToDeg("nudge", (s.attached ? usToDeg(sel, s.us) : s.restDeg) + kNudgeDeg); break;
        case '[': moveToDeg("nudge", (s.attached ? usToDeg(sel, s.us) : s.restDeg) - kNudgeDeg); break;

        case 'm':
            if (!s.attached) { Serial.println("  ! drive the servo first"); break; }
            s.markMin = (int16_t)lroundf(usToDeg(sel, s.us));
            Serial.printf("  %s: marked MIN = %d deg (%u us)\n", s.name, s.markMin, s.us);
            break;

        case 'M':
            if (!s.attached) { Serial.println("  ! drive the servo first"); break; }
            s.markMax = (int16_t)lroundf(usToDeg(sel, s.us));
            Serial.printf("  %s: marked MAX = %d deg (%u us)\n", s.name, s.markMax, s.us);
            break;

        case 'z':
            s.markMin = s.markMax = -1;
            Serial.printf("  %s: marks cleared\n", s.name);
            break;

        case 'k': printPaste();            break;
        case 'p': showServo(sel, "");      break;
        case 'P': printAll();              break;
        case 'd': detach(sel); showServo(sel, "detached"); break;
        case 'D':
            for (uint8_t i = 0; i < SERVO_COUNT; i++) detach(i);
            Serial.println("  all servos detached");
            break;
        case 'h': printHelp();             break;
        default:  break;
    }
}

void runLine(const char *s)
{
    switch (s[0])
    {
        case 'a': moveToDeg("angle", atof(s + 1));          break;
        case 'u': moveToUs ("pulse", (int32_t)atol(s + 1)); break;
        default:
            Serial.printf("  ? unknown command \"%s\"  (h for help)\n", s);
            break;
    }
}

bool isInstantKey(char c)
{
    return (c >= '1' && c <= '5') ||
           c == 's' || c == 't' || c == 'c' || c == '+' || c == '-' ||
           c == ']' || c == '[' || c == 'm' || c == 'M' || c == 'z' ||
           c == 'k' || c == 'p' || c == 'P' || c == 'd' || c == 'D' || c == 'h';
}

// ── Arduino ─────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    if (Pins::Servos::kTcaChannel != 255)
        pwm.attachMux(&benchMux, Pins::Servos::kTcaChannel);

    pwm.setOscillatorHz(kOscHz);
    if (!pwm.begin(kServoFreqHz))
        Serial.printf("  ! no PCA9685 at 0x%02X on Wire%u (SDA %u / SCL %u), TCA ch %u -- check wiring\n",
                      kPcaAddress, Pins::Servos::kI2cBus,
                      Pins::Servos::kI2cSda, Pins::Servos::kI2cScl, Pins::Servos::kTcaChannel);

    // Silence every servo channel before letting pulses out, so nothing jumps.
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        detach(i);
    pwm.enableOutputs();

    printHelp();
}

void loop()
{
    while (Serial.available())
    {
        const char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (lineLen)
            {
                lineBuf[lineLen] = '\0';
                runLine(lineBuf);
                lineLen = 0;
            }
            continue;
        }

        if (c == ' ' || c == '\t')
            continue;

        // A single-key command only counts when nothing is half-typed, so the
        // digits in "a132" or "u1500" are not read as servo picks.
        if (lineLen == 0 && isInstantKey(c))
        {
            runInstant(c);
            continue;
        }

        if (lineLen < sizeof(lineBuf) - 1)
            lineBuf[lineLen++] = c;
        else
            lineLen = 0; // overflow: drop the garbage rather than truncate
    }
}
