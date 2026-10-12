/*
 * 02_pca9685_channel_test — stage 2: one servo through the PCA9685
 *
 *   Drives ONE PCA9685 channel directly with lib/PCA9685,
 *   deliberately bypassing ServoSystem. 
 * 
 * calibration procedure  
 *   1.  #<n>   select the channel the servo is on (or set kStartChannel).
 *   2.  ] / [  walk toward one mechanical stop, one degree at a time.
 *              Stop the moment it touches WITHOUT straining or buzzing.
 *   3.  m      mark that angle as this servo's minAngleDeg.
 *   4.  ] / [  walk to the other stop the same way.
 *   5.  M      mark that angle as maxAngleDeg.
 *   6.  k      print the paste-ready kCalib row -> constants.h.
 *   7.  a<deg> jump to the angles you want for home / deploy / neutral /
 *              open / closed, and paste those with k as well.
 *
 *   keys
 *   instant (no Enter needed)
 *     s  safe angle        c  center angle       t  test angle
 *     +  +25 us            -  -25 us
 *     ]  +1 deg            [  -1 deg
 *     m  mark MIN angle    M  mark MAX angle     z  clear marks
 *     k  print paste-ready constants
 *     p  print position    d  detach (stop pulsing)   h  help
 *
 *   typed, then Enter
 *     a<deg>   go to an angle       e.g.  a90    a132.5
 *     u<us>    go to a pulse width  e.g.  u1500
 *     #<ch>    switch channel 0-15  e.g.  #3
 */

#include <Arduino.h>
#include "PCA9685.hpp"
#include "TCA9548A/TCA9548A.h"
#include "pins.h"
#include "constants.h"

// ── Test configuration (edit here only) ─────────────────────────────────
constexpr uint8_t  kPcaAddress   = Pins::Servos::kPcaI2cAddress; // board + bus from pins.h
constexpr uint8_t  kStartChannel = 12;        // channel selected at boot
constexpr uint32_t kOscHz        = Constants::ServoConfig::kPcaOscillatorHz;
constexpr float    kServoFreqHz  = 50.0f;     // hobby-servo frame rate

// The pulse <-> angle map. MUST match kCalib[i].minPulseUs / maxPulseUs
// in include/constants.h for this servo, or the angles printed here are
// not the angles ServoSystem will drive.
constexpr uint16_t kPulseAt0Deg   = 500;
constexpr uint16_t kPulseAt180Deg = 2500;

// Presets, expressed as angles now — that is what goes into constants.h.
constexpr float kSafeDeg   = 50.0f;
constexpr float kCenterDeg = 90.0f;
constexpr float kTestDeg   = 120.0f;

constexpr uint16_t kNudgeUs  = 25;    // step for + / -
constexpr float    kNudgeDeg = 1.0f;  // step for ] / [

constexpr uint16_t kMinUs = 500;      // hard guard rails, never exceeded
constexpr uint16_t kMaxUs = 2500;
// ────────────────────────────────────────────────────────────────────────

// static + own name: instances.cpp (always linked) already defines i2cMux
static TCA9548A benchMux(Pins::I2cMux::kAddress, PCA9685::busFromIndex(Pins::I2cMux::kBus));
static PCA9685  pwm(kPcaAddress, PCA9685::busFromIndex(Pins::Servos::kI2cBus), Pins::Servos::kOePin);

static uint8_t  channel   = kStartChannel;
static uint16_t currentUs = 1500;
static bool     attached  = true;

// Marked mechanical stops, in degrees. -1 = not marked yet.
static int16_t markMinDeg = -1;
static int16_t markMaxDeg = -1;

static char lineBuf[16];
static uint8_t lineLen = 0;

// ── Pulse <-> angle ─────────────────────────────────────────────────────
float usToDeg(uint16_t us)
{
    return (float)((int32_t)us - (int32_t)kPulseAt0Deg) * 180.0f /
           (float)((int32_t)kPulseAt180Deg - (int32_t)kPulseAt0Deg);
}

uint16_t degToUs(float deg)
{
    float us = (float)kPulseAt0Deg +
               deg * (float)((int32_t)kPulseAt180Deg - (int32_t)kPulseAt0Deg) / 180.0f;
    if (us < (float)kMinUs) us = (float)kMinUs;
    if (us > (float)kMaxUs) us = (float)kMaxUs;
    return (uint16_t)lroundf(us);
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
void showPosition(const char *tag)
{
    const float deg = usToDeg(currentUs);

    Serial.printf("  ch %-2u  %4u us  ", channel, currentUs);
    printDeg(deg);
    Serial.print(" deg");

    if (markMinDeg >= 0 || markMaxDeg >= 0)
    {
        Serial.print("   marks[");
        if (markMinDeg >= 0) Serial.printf("min %d", markMinDeg); else Serial.print("min --");
        Serial.print(" | ");
        if (markMaxDeg >= 0) Serial.printf("max %d", markMaxDeg); else Serial.print("max --");
        Serial.print("]");

        // A quiet warning that you have walked outside the band you marked.
        const int16_t lo = (markMinDeg >= 0) ? markMinDeg : 0;
        const int16_t hi = (markMaxDeg >= 0) ? markMaxDeg : 180;
        if (deg < (float)lo - 0.5f || deg > (float)hi + 0.5f)
            Serial.print("  <-- OUTSIDE marked band");
    }

    if (!attached) Serial.print("   (DETACHED — no pulses)");
    if (tag && *tag) Serial.printf("   %s", tag);
    Serial.println();
}

void moveToUs(const char *tag, int32_t us)
{
    if (us < (int32_t)kMinUs) us = kMinUs;
    if (us > (int32_t)kMaxUs) us = kMaxUs;

    currentUs = (uint16_t)us;
    attached  = true;
    pwm.writeMicroseconds(channel, currentUs);
    showPosition(tag);
}

void moveToDeg(const char *tag, float deg)
{
    if (deg <   0.0f) deg =   0.0f;
    if (deg > 180.0f) deg = 180.0f;
    moveToUs(tag, degToUs(deg));
}

void detach()
{
    pwm.off(channel); // no pulses at all — servo goes limp
    attached = false;
    showPosition("detached");
}

void selectChannel(int ch)
{
    if (ch < 0 || ch > 15)
    {
        Serial.printf("  ! channel %d out of range (0-15)\n", ch);
        return;
    }
    // Stop pulsing the channel we are leaving, so it does not keep holding.
    pwm.off(channel);
    channel  = (uint8_t)ch;
    attached = false;
    Serial.printf("  channel -> %u  (previous channel released, this one is\n"
                  "    still silent — press s / c / a<deg> to start driving it)\n",
                  channel);
}

// ── Paste-ready output ──────────────────────────────────────────────────
void printPaste()
{
    Serial.println();
    Serial.println("// ─── paste into Constants::ServoConfig::kCalib[] (include/constants.h) ───");

    if (markMinDeg < 0 || markMaxDeg < 0)
    {
        Serial.println("//  ! stops not marked yet — walk to each mechanical stop and press");
        Serial.println("//    m (min) and M (max). Placeholder limits shown below.");
    }

    const int lo = (markMinDeg >= 0) ? markMinDeg : 0;
    const int hi = (markMaxDeg >= 0) ? markMaxDeg : 180;
    const int a  = (lo <= hi) ? lo : hi;   // kCalib wants min first
    const int b  = (lo <= hi) ? hi : lo;

    Serial.printf("{ Pins::Servos::k????Ch, %u, %u, %d, %d },   // channel %u\n",
                  kPulseAt0Deg, kPulseAt180Deg, a, b, channel);

    Serial.println("// ─── current position, as a position constant ───");
    Serial.printf("static constexpr uint8_t k????Deg = %ld;   // ",
                  (long)lroundf(usToDeg(currentUs)));
    printDeg(usToDeg(currentUs));
    Serial.printf(" deg = %u us on channel %u\n", currentUs, channel);
    Serial.println();
}

void printHelp()
{
    Serial.println();
    Serial.println("[02_pca9685_channel_test] one PCA9685 channel, in degrees");
    Serial.printf("  addr=0x%02X  channel=%u  freq=%.0f Hz\n",
                  kPcaAddress, channel, kServoFreqHz);
    Serial.printf("  map: %u us = 0 deg   %u us = 180 deg   (must match kCalib)\n",
                  kPulseAt0Deg, kPulseAt180Deg);
    Serial.println("  instant keys:");
    Serial.println("    s c t   safe / center / test angle");
    Serial.println("    + -     pulse -/+ 25 us        ] [   angle +/- 1 deg");
    Serial.println("    m M     mark MIN / MAX angle   z     clear marks");
    Serial.println("    k       print paste-ready constants");
    Serial.println("    p       print position        d  detach     h  help");
    Serial.println("  typed, then Enter:");
    Serial.println("    a<deg>  go to angle    u<us>  go to pulse    #<ch>  channel 0-15");
    showPosition("");
}

// ── Input ───────────────────────────────────────────────────────────────
void runInstant(char c)
{
    switch (c)
    {
        case 's': moveToDeg("safe",   kSafeDeg);   break;
        case 'c': moveToDeg("center", kCenterDeg); break;
        case 't': moveToDeg("test",   kTestDeg);   break;

        case '+': moveToUs("nudge", (int32_t)currentUs + kNudgeUs); break;
        case '-': moveToUs("nudge", (int32_t)currentUs - kNudgeUs); break;

        case ']': moveToDeg("nudge", usToDeg(currentUs) + kNudgeDeg); break;
        case '[': moveToDeg("nudge", usToDeg(currentUs) - kNudgeDeg); break;

        case 'm':
            markMinDeg = (int16_t)lroundf(usToDeg(currentUs));
            Serial.printf("  marked MIN = %d deg (%u us)\n", markMinDeg, currentUs);
            break;

        case 'M':
            markMaxDeg = (int16_t)lroundf(usToDeg(currentUs));
            Serial.printf("  marked MAX = %d deg (%u us)\n", markMaxDeg, currentUs);
            break;

        case 'z':
            markMinDeg = markMaxDeg = -1;
            Serial.println("  marks cleared");
            break;

        case 'k': printPaste();      break;
        case 'p': showPosition("");  break;
        case 'd': detach();          break;
        case 'h': printHelp();       break;
        default:  break;
    }
}

void runLine(const char *s)
{
    switch (s[0])
    {
        case 'a': moveToDeg("angle", atof(s + 1));            break;
        case 'u': moveToUs ("pulse", (int32_t)atol(s + 1));   break;
        case '#': selectChannel(atoi(s + 1));                 break;
        default:
            Serial.printf("  ? unknown command \"%s\"  (h for help)\n", s);
            break;
    }
}

bool isInstantKey(char c)
{
    return c == 's' || c == 'c' || c == 't' || c == '+' || c == '-' ||
           c == ']' || c == '[' || c == 'm' || c == 'M' || c == 'z' ||
           c == 'k' || c == 'p' || c == 'd' || c == 'h';
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
    pwm.enableOutputs();

    printHelp();
    moveToDeg("safe", kSafeDeg);
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

        // A known single-key command only counts as one when nothing is
        // half-typed, so "a132" is not eaten by the 'a'... (there is no
        // instant 'a', but the same rule keeps '-' out of typed numbers).
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
