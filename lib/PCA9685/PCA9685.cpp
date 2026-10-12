/**
  @file PCA9685.cpp
  @date 2026-09-21

  @brief Implementation of the generic PCA9685 servo/PWM driver.
**/

#include "PCA9685.hpp"
#include "TCA9548A/TCA9548A.h"

TwoWire &PCA9685::busFromIndex(uint8_t bus)
{
    switch (bus)
    {
        case 1:  return Wire1;
        case 2:  return Wire2;
        default: return Wire;
    }
}

PCA9685::PCA9685(uint8_t address, TwoWire &wire, uint8_t oePin)
    : _wire(wire)
    , _addr(address)
    , _oePin(oePin)
    , _oscHz(kDefaultOscHz)
    , _prescale(0x1E) // chip power-on default (~200 Hz)
    , _ok(false)
    , _mux(nullptr)
    , _muxChannel(0)
{
}

void PCA9685::attachMux(TCA9548A *mux, uint8_t channel)
{
    _mux        = mux;
    _muxChannel = channel;
}

void PCA9685::_select()
{
    if (_mux)
        _mux->selectChannel(_muxChannel); // no-op if already open
}

// Lifecycle

bool PCA9685::begin(float freqHz, uint32_t i2cClockHz)
{
    // Hold the outputs off first, before touching the chip at all.
    if (_oePin != kNoPin)
    {
        pinMode(_oePin, OUTPUT);
        digitalWrite(_oePin, HIGH); // OE is active LOW
    }

    _wire.begin();
    if (i2cClockHz)
        _wire.setClock(i2cClockHz);

    _ok = isConnected();
    if (!_ok)
        return false;

    // Asleep, auto-increment on, ALLCALL off (frees 0x70 for the TCA9548A).
    _write8(REG_MODE1, MODE1_SLEEP | MODE1_AI);
    _write8(REG_MODE2, MODE2_OUTDRV);

    allOff();              // nothing pulses until someone asks for it
    setFrequency(freqHz);  // also wakes the chip
    return true;
}

bool PCA9685::isConnected()
{
    _select();
    _wire.beginTransmission(_addr);
    return _wire.endTransmission() == 0;
}

// Frequency

void PCA9685::setFrequency(float freqHz)
{
    if (freqHz < 24.0f)   freqHz = 24.0f;   // datasheet range
    if (freqHz > 1526.0f) freqHz = 1526.0f;

    // prescale = round(osc / (4096 * f)) - 1
    float p = ((float)_oscHz / (4096.0f * freqHz)) + 0.5f - 1.0f;
    if (p < 3.0f)   p = 3.0f;
    if (p > 255.0f) p = 255.0f;
    _prescale = (uint8_t)p;

    const uint8_t mode = (uint8_t)((_read8(REG_MODE1) & ~MODE1_RESTART & ~MODE1_ALLCALL) | MODE1_AI);

    _write8(REG_MODE1, mode | MODE1_SLEEP);     // prescaler only writable asleep
    _write8(REG_PRESCALE, _prescale);
    _write8(REG_MODE1, mode & ~MODE1_SLEEP);    // wake
    delayMicroseconds(500);                     // oscillator settle time
    _write8(REG_MODE1, (mode & ~MODE1_SLEEP) | MODE1_RESTART);
}

float PCA9685::frequency() const
{
    return (float)_oscHz / (4096.0f * ((float)_prescale + 1.0f));
}

// Output

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel >= kChannels)
        return;
    _write4(REG_LED0_ON_L + 4 * channel, on, off);
}

uint16_t PCA9685::usToTicks(uint16_t us) const
{
    // ticks = us * f * 4096 / 1e6, with f = osc / (4096 * (prescale + 1))
    const uint64_t num = (uint64_t)us * (uint64_t)_oscHz;
    const uint64_t den = (uint64_t)1000000ULL * ((uint64_t)_prescale + 1ULL);
    uint32_t ticks = (uint32_t)((num + den / 2) / den);
    if (ticks > kFullScale - 1)
        ticks = kFullScale - 1;
    return (uint16_t)ticks;
}

void PCA9685::writeMicroseconds(uint8_t channel, uint16_t us)
{
    if (us == 0)
    {
        off(channel);
        return;
    }
    setPWM(channel, 0, usToTicks(us));
}

void PCA9685::off(uint8_t channel)
{
    setPWM(channel, 0, kFullScale); // bit 12 of OFF = full OFF
}

void PCA9685::allOff()
{
    _write4(REG_ALL_LED_ON_L, 0, kFullScale);
}

void PCA9685::sleep()
{
    _write8(REG_MODE1, _read8(REG_MODE1) | MODE1_SLEEP);
}

void PCA9685::wake()
{
    const uint8_t mode = _read8(REG_MODE1);
    _write8(REG_MODE1, mode & ~MODE1_SLEEP);
    delayMicroseconds(500);
    if (mode & MODE1_RESTART)
        _write8(REG_MODE1, (mode & ~MODE1_SLEEP) | MODE1_RESTART);
}

void PCA9685::enableOutputs()
{
    if (_oePin != kNoPin)
        digitalWrite(_oePin, LOW);
}

void PCA9685::disableOutputs()
{
    if (_oePin != kNoPin)
        digitalWrite(_oePin, HIGH);
}

// I2C

bool PCA9685::_write8(uint8_t reg, uint8_t value)
{
    _select();
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(value);
    return _wire.endTransmission() == 0;
}

uint8_t PCA9685::_read8(uint8_t reg)
{
    _select();
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0)
        return 0;
    if (_wire.requestFrom(_addr, (uint8_t)1) != 1)
        return 0;
    return (uint8_t)_wire.read();
}

void PCA9685::_write4(uint8_t reg, uint16_t on, uint16_t off)
{
    _select();
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write((uint8_t)(on & 0xFF));
    _wire.write((uint8_t)(on >> 8));
    _wire.write((uint8_t)(off & 0xFF));
    _wire.write((uint8_t)(off >> 8));
    _wire.endTransmission();
}
