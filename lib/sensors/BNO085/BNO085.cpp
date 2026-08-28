/*
 * @file bno085.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the BNO085 class.
 * @version 0.1
 * @date 2026-01-12
 */

#include "BNO085/BNO085.hpp"
#include <math.h>

// Debugged config (see src/test_sensors/bno085_standalone.cpp): the chip is
// wired to the Teensy 4.1's second I2C bus (Wire2: SDA2=25, SCL2=24), not
// the main Wire. 100kHz never completed the SH-2 handshake ("I2C address
// not found" in begin_I2C) -- 50kHz is the validated frequency.
static constexpr uint8_t  kAddressPrimary   = 0x4A; // ADR to GND
static constexpr uint8_t  kAddressSecondary = 0x4B; // ADR to VCC
static constexpr uint32_t kI2cFrequency     = 50000;
static constexpr uint32_t REPORT_PERIOD_US  = 20000; // 50 Hz, matches bno085_standalone.cpp

// begin_I2C()'s own internal hardwareReset() only holds RST for ~30ms, which
// isn't enough for this chip's SH-2 firmware to boot on this board -- the
// handshake fails even though the bus ACKs fine. We reset RST ourselves with
// a generous startup margin and never let the library touch the pin (the
// Adafruit_BNO08x default constructor below is never given a reset pin).
static constexpr uint8_t  kRstPin              = Pins::kBnoRstReserved; // see pins.h: possible SERVO5 conflict, confirmed as BNO RST for now
static constexpr uint8_t  kAddressWaitAttempts = 20;
static constexpr uint32_t kAddressWaitDelayMs  = 100;
static constexpr uint8_t  kBeginAttempts       = 5;
static constexpr uint32_t kBeginRetryDelayMs   = 300;
static constexpr uint32_t kResetStartupDelayMs = 500;
static constexpr uint32_t kStallTimeoutMs      = 1500;

// Without RST physically wired, a manual reset can't actually un-wedge the
// chip -- retrying forever here would hang the whole robot (BNO085::update()
// is called every Drive::update() tick). Give up after this many reset
// cycles instead of blocking indefinitely.
static constexpr uint8_t kMaxConnectCycles = 5;

BNO085::BNO085() : initialized(false)
{
}

void BNO085::manualReset()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, HIGH);
    delay(10);
    digitalWrite(kRstPin, LOW);
    delay(10);
    digitalWrite(kRstPin, HIGH);
    delay(10);

    delay(kResetStartupDelayMs); // SH-2 firmware startup margin
}

bool BNO085::waitForAddress(uint8_t address)
{
    for (uint8_t attempt = 0; attempt < kAddressWaitAttempts; ++attempt)
    {
        Wire2.beginTransmission(address);
        if (Wire2.endTransmission() == 0)
            return true;

        delay(kAddressWaitDelayMs);
    }
    return false;
}

bool BNO085::tryAddress(uint8_t address)
{
    if (!waitForAddress(address))
        return false;

    for (uint8_t attempt = 0; attempt < kBeginAttempts; ++attempt)
    {
        if (bno.begin_I2C(address, &Wire2))
            return true;

        delay(kBeginRetryDelayMs);
    }
    return false;
}

bool BNO085::enableReports()
{
    if (!bno.enableReport(SH2_ROTATION_VECTOR, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable ROTATION_VECTOR report");
        return false;
    }

    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable LINEAR_ACCELERATION report");
        return false;
    }

    return true;
}

bool BNO085::begin()
{
    Serial.println("BEGIN >>>> BNO085 <<<<");

    Wire2.begin();
    Wire2.setClock(kI2cFrequency);

    manualReset();

    bool connected = false;
    for (uint8_t cycle = 0; cycle < kMaxConnectCycles && !connected; ++cycle)
    {
        connected = tryAddress(kAddressPrimary) || tryAddress(kAddressSecondary);

        if (!connected)
        {
            Serial.println("Failed to BEGIN >>>> BNO085 <<<< (0x4A/0x4B), resetting and retrying...");
            manualReset();
        }
    }

    if (!connected)
    {
        Serial.println(">>>> BNO085 <<<< giving up (chip not responding on 0x4A/0x4B after several reset cycles)");
        return false;
    }

    Serial.println(">>>> BNO085 <<<< BEGINED successfully");

    if (!enableReports())
        return false;

    initialized     = true;
    firstYawSample_ = true;
    filteredYawDeg_ = 0.0f;
    lastValidMs_    = millis();

    return true;
}

void BNO085::recover()
{
    Serial.println("BNO085: stalled, forcing reset+reinit");

    bool connected = false;
    manualReset();
    for (uint8_t cycle = 0; cycle < kMaxConnectCycles && !connected; ++cycle)
    {
        connected = tryAddress(kAddressPrimary) || tryAddress(kAddressSecondary);
        if (!connected)
            manualReset();
    }

    if (!connected)
    {
        Serial.println("BNO085: lost for good (no RST wired to un-wedge it) -- giving up, yaw stays frozen");
        initialized = false;
        return;
    }

    enableReports();
    lastValidMs_ = millis();
}

void BNO085::update()
{
    if (!initialized)
        return;

    sh2_SensorValue_t val;
    if (!bno.getSensorEvent(&val))
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

    if (val.sensorId == SH2_ROTATION_VECTOR)
    {
        float qw = val.un.rotationVector.real;
        float qx = val.un.rotationVector.i;
        float qy = val.un.rotationVector.j;
        float qz = val.un.rotationVector.k;

        float rawYawDeg, rawPitchDeg, rawRollDeg;
        quaternionToEuler(qw, qx, qy, qz, rawYawDeg, rawPitchDeg, rawRollDeg);

        pitchDeg_ = rawPitchDeg;
        rollDeg_  = rawRollDeg;

        static constexpr float alpha = 0.07f;

        float rawNeg = -wrapAngle(rawYawDeg);

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