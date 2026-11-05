/*
 * @file bno085.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the BNO085 class.
 * @version 0.1
 * @date 2026-01-12
 */

#include "BNO085/BNO085.hpp"
#include <math.h>
#include "I2cRecovery.hpp"

// Debugged config (see src/test_sensors/bno085_standalone.cpp): the chip is
// wired to the Teensy 4.1's second I2C bus (Wire2: SDA2=25, SCL2=24), not
// the main Wire. 100kHz never completed the SH-2 handshake ("I2C address
// not found" in begin_I2C) -- 50kHz is the validated frequency.
static constexpr uint8_t  kAddressPrimary   = 0x4A; // ADR to GND
static constexpr uint8_t  kAddressSecondary = 0x4B; // ADR to VCC
static constexpr uint32_t kI2cFrequency     = 50000;
static constexpr uint32_t REPORT_PERIOD_US  = 20000; // 50 Hz, matches bno085_standalone.cpp

// We drive RST ourselves (the Adafruit_BNO08x object is never given the pin):
// its own reset is too short for this chip, and while RST is held LOW the chip
// cannot hold SDA, which is the moment to free Wire2 after a Teensy-only reset.
static constexpr uint8_t  kRstPin              = Pins::kBnoRstReserved; // pin 5; 255 = no reset pin (see pins.h)
static constexpr uint8_t  kAddressWaitAttempts = 20; // begin(): up to 2 s per address
static constexpr uint32_t kAddressWaitDelayMs  = 100;
static constexpr uint8_t  kBeginAttempts       = 5;
static constexpr uint32_t kBeginRetryDelayMs   = 300;
static constexpr uint32_t kResetStartupDelayMs = 500;
static constexpr uint32_t kStartupResetLowMs   = 2000; // first begin(): total RST LOW, counted from holdInReset()
static constexpr uint32_t kStallTimeoutMs      = 1500;
static constexpr uint8_t  kMaxEventsPerUpdate  = 8; // drain the queue so the yaw is never stale

// BNO085::update() runs every Drive::update() tick, so a mid-run recover()
// blocks the robot: it gets fewer and shorter cycles than the startup begin(),
// which retries until the chip answers (the robot is not driving yet).
static constexpr uint8_t kRecoverCycles          = 2;
static constexpr uint8_t kRecoverAddressAttempts = 5; // 0.5 s per address

// millis() when holdInReset() pulled RST LOW, 0 = not held
static uint32_t sResetHeldSinceMs = 0;

BNO085::BNO085() : initialized(false)
{
}

void BNO085::holdInReset()
{
    if (kRstPin == 255)
        return;
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    sResetHeldSinceMs = millis();
    if (sResetHeldSinceMs == 0)
        sResetHeldSinceMs = 1;
}

// RST LOW for lowMs in total, counting the time already held by holdInReset().
void BNO085::manualReset(uint32_t lowMs)
{
    if (kRstPin != 255) {
        if (sResetHeldSinceMs == 0)
            holdInReset();
        while (millis() - sResetHeldSinceMs < lowMs)
            ;
        sResetHeldSinceMs = 0;
    }

    // A Teensy reset (e.g. an upload) does not power-cycle the BNO, which can be left
    // holding SDA mid-transfer; held in reset it lets go, so free and restart Wire2 here.
    if (!recoverI2cBus(Pins::kBnoSda, Pins::kBnoScl))
        log("BNO085: SDA still held low after bus recovery");
    Wire2.begin();
    Wire2.setClock(kI2cFrequency);

    if (kRstPin != 255)
        digitalWrite(kRstPin, HIGH);
    log("BNO085: RST released");

    delay(kResetStartupDelayMs); // SH-2 firmware startup margin
}

bool BNO085::waitForAddress(uint8_t address, uint8_t attempts)
{
    for (uint8_t attempt = 0; attempt < attempts; ++attempt)
    {
        Wire2.beginTransmission(address);
        if (Wire2.endTransmission() == 0)
            return true;

        delay(kAddressWaitDelayMs);
    }
    return false;
}

bool BNO085::tryAddress(uint8_t address, uint8_t addressAttempts)
{
    if (!waitForAddress(address, addressAttempts))
        return false;

    for (uint8_t attempt = 0; attempt < kBeginAttempts; ++attempt)
    {
        if (bno.begin_I2C(address, &Wire2))
            return true;

        delay(kBeginRetryDelayMs);
    }
    return false;
}

void BNO085::log(const char *msg) const
{
    if (verbose_)
        Serial.println(msg);
}

bool BNO085::enableReports()
{
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_PERIOD_US)) // no magnetometer: motors bend its heading
    {
        log("Failed to enable GAME_ROTATION_VECTOR report");
        return false;
    }

    // Linear acceleration is not used: enabling it halved the yaw events read per update.

    // Dynamic calibration is lost on every reset; the gyro one removes the zero-rate
    // offset while the robot stands still, which otherwise makes the yaw drift.
    if (sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG) != SH2_OK)
        log("BNO085: failed to enable dynamic calibration");

    return true;
}

// Blocks until the chip answers: it only runs at startup, before the robot drives.
bool BNO085::begin()
{
    log("BEGIN >>>> BNO085 <<<<");

    // Only the first reset is long; the retries and recover() keep the short pulse.
    manualReset(kStartupResetLowMs);

    while (!((tryAddress(kAddressPrimary, kAddressWaitAttempts) ||
              tryAddress(kAddressSecondary, kAddressWaitAttempts)) &&
             enableReports()))
    {
        log("Failed to BEGIN >>>> BNO085 <<<< (0x4A/0x4B), resetting and retrying...");
        manualReset();
    }

    log(">>>> BNO085 <<<< BEGINED successfully");

    initialized     = true;
    firstYawSample_ = true;
    filteredYawDeg_ = 0.0f;
    yawOffsetDeg_   = 0.0f;
    reanchorYaw_    = false;
    lastValidMs_    = millis();

    return true;
}

void BNO085::recover()
{
    log("BNO085: stalled, forcing reset+reinit");

    bool connected = false;
    for (uint8_t cycle = 0; cycle < kRecoverCycles && !connected; ++cycle)
    {
        manualReset();
        connected = tryAddress(kAddressPrimary, kRecoverAddressAttempts) ||
                    tryAddress(kAddressSecondary, kRecoverAddressAttempts);
    }

    if (!connected)
    {
        log("BNO085: lost for good after hardware resets -- giving up, yaw stays frozen");
        initialized = false;
        return;
    }

    enableReports();
    reanchorYaw_ = true;
    lastValidMs_ = millis();
}

void BNO085::update()
{
    if (!initialized)
        return;

    // A brownout (e.g. the elevator motor reversing) resets the chip and disables its reports.
    if (bno.wasReset())
    {
        log("BNO085: chip reset, re-enabling reports");
        enableReports();
        reanchorYaw_ = true;
        lastValidMs_ = millis();
    }

    sh2_SensorValue_t val;
    bool gotEvent = false;
    for (uint8_t i = 0; i < kMaxEventsPerUpdate && bno.getSensorEvent(&val); ++i)
    {
        gotEvent = true;
        handleEvent(val);
    }

    if (!gotEvent)
    {
        // The chip occasionally stops sending reports without the bus itself
        // going down -- nothing ever overwrites the last decoded value, so
        // force a full reset+reinit once we've gone too long without one.
        // recover() gives up (and sets initialized=false) instead of
        // blocking forever if the chip never comes back.
        if (millis() - lastValidMs_ > kStallTimeoutMs)
            recover();

        return;
    }

    lastValidMs_ = millis();
}

void BNO085::handleEvent(const sh2_SensorValue_t &val)
{
    if (val.sensorId == SH2_GAME_ROTATION_VECTOR)
    {
        float qw = val.un.gameRotationVector.real;
        float qx = val.un.gameRotationVector.i;
        float qy = val.un.gameRotationVector.j;
        float qz = val.un.gameRotationVector.k;

        float rawYawDeg, rawPitchDeg, rawRollDeg;
        quaternionToEuler(qw, qx, qy, qz, rawYawDeg, rawPitchDeg, rawRollDeg);

        pitchDeg_ = rawPitchDeg;
        rollDeg_  = rawRollDeg;

        static constexpr float alpha = 0.30f; // 0.07 added ~0.3 s of lag and made the yaw hold oscillate

        float rawNeg = -wrapAngle(rawYawDeg);

        if (reanchorYaw_ && !firstYawSample_)
            yawOffsetDeg_ = wrapAngle(filteredYawDeg_ - rawNeg);
        reanchorYaw_ = false;
        rawNeg = wrapAngle(rawNeg + yawOffsetDeg_);

        if (firstYawSample_)
        {
            filteredYawDeg_ = rawNeg;
            firstYawSample_ = false;
        }
        else
        {
            float err       = wrapAngle(rawNeg - filteredYawDeg_);
            filteredYawDeg_ = wrapAngle(filteredYawDeg_ + alpha * err);
        }
    }

    if (val.sensorId == SH2_LINEAR_ACCELERATION)
    {
        linAccX_ = val.un.linearAcceleration.x;
        linAccY_ = val.un.linearAcceleration.y;
        linAccZ_ = val.un.linearAcceleration.z;
    }
}

void BNO085::quaternionToEuler(float qw, float qx, float qy, float qz,
                                float &yaw, float &pitch, float &roll) const
{
    yaw   = atan2f(2.0f * (qw * qz + qx * qy),
                   1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / M_PI);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp       = fmaxf(-1.0f, fminf(1.0f, sinp));
    pitch      = asinf(sinp) * (180.0f / M_PI);

    roll  = atan2f(2.0f * (qw * qx + qy * qz),
                   1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / M_PI);
}

float BNO085::wrapAngle(float angle) const
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f)
        angle += 360.0f;

    if (angle > 180.0f)
        angle -= 360.0f;
    else if (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

float BNO085::getYaw() const
{
    return filteredYawDeg_ * (M_PI / 180.0f);
}

float BNO085::getRoll() const
{
    return rollDeg_;
}

float BNO085::getPitch() const
{
    return pitchDeg_;
}

std::tuple<float, float, float> BNO085::getLinealAcceleration()
{
    update();
    return std::make_tuple(linAccX_, linAccY_, linAccZ_);
}

void BNO085::getAngular()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    (void)current_yaw;
    (void)current_roll;
    (void)current_pitch;
}

void BNO085::getAngularPrinted()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    Serial.println("CURRENT ORIENTATION:");
    Serial.print("  Yaw (Z-axis):   ");
    Serial.print(current_yaw * 180.0f / M_PI, 2);
    Serial.println(" deg");
    Serial.print("  Roll (Y-axis):  ");
    Serial.print(current_roll, 2);
    Serial.println(" deg");
    Serial.print("  Pitch (X-axis): ");
    Serial.print(current_pitch, 2);
    Serial.println(" deg");
}