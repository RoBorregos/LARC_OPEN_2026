/**
  @file PCA9685.hpp
  @date 2026-09-21

  @brief Minimal driver for the PCA9685 16-channel, 12-bit PWM board,
   written for hobby servos. No Adafruit.
**/

#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <Arduino.h>
#include <Wire.h>

class TCA9548A;

class PCA9685
{
public:
    static constexpr uint8_t  kChannels      = 16;
    static constexpr uint8_t  kNoPin         = 255;
    static constexpr uint32_t kDefaultOscHz  = 25000000UL;
    static constexpr uint16_t kFullScale     = 4096; // ticks per PWM period

    // bus is one of 0 = Wire, 1 = Wire1, 2 = Wire2 (Teensy 4.1).
    static TwoWire &busFromIndex(uint8_t bus);

    explicit PCA9685(uint8_t address = 0x40, TwoWire &wire = Wire,
                     uint8_t oePin = kNoPin);

    // The chip sits behind a TCA9548A: select `channel` (0-7) before every
    // transaction. Pass the SAME TCA9548A object the rest of the robot uses
    // (it caches the open channel). nullptr detaches.
    void attachMux(TCA9548A *mux, uint8_t channel);

    // Starts the bus, checks the chip answers, configures it and sets the
    // PWM frequency. Leaves every channel OFF and outputs disabled (if an
    // OE pin is wired). i2cClockHz = 0 leaves the bus clock untouched.
    // Returns false if the chip did not ACK.
    bool begin(float freqHz = 50.0f, uint32_t i2cClockHz = 0);

    // True if the chip ACKs its address right now.
    bool isConnected();
    bool ok() const { return _ok; }

    // Call BEFORE begin() (or call setFrequency() again afterwards).
    void setOscillatorHz(uint32_t oscHz) { _oscHz = oscHz; }
    uint32_t oscillatorHz() const { return _oscHz; }

    // Reprograms the prescaler (the chip sleeps briefly while doing it).
    void  setFrequency(float freqHz);
    float frequency() const; // actual frequency after prescaler rounding

    // Raw 12-bit PWM: output goes high at tick `on`, low at tick `off`.
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    // Servo-style pulse width in microseconds (0 = channel off).
    void     writeMicroseconds(uint8_t channel, uint16_t us);
    uint16_t usToTicks(uint16_t us) const;

    void off(uint8_t channel); // full OFF: no pulses, servo goes limp
    void allOff();             // every channel full OFF in one write

    void sleep();  // oscillator off, all outputs stop
    void wake();   // resumes the previous PWM settings

    // OE pin control. No-ops when no OE pin is configured.
    void enableOutputs();
    void disableOutputs();

    uint8_t  address() const { return _addr; }
    uint8_t  prescale() const { return _prescale; }

private:
    // Registers
    static constexpr uint8_t REG_MODE1        = 0x00;
    static constexpr uint8_t REG_MODE2        = 0x01;
    static constexpr uint8_t REG_LED0_ON_L    = 0x06;
    static constexpr uint8_t REG_ALL_LED_ON_L = 0xFA;
    static constexpr uint8_t REG_PRESCALE     = 0xFE;

    // MODE1 bits
    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_AI      = 0x20; // register auto-increment
    static constexpr uint8_t MODE1_SLEEP   = 0x10;
    static constexpr uint8_t MODE1_ALLCALL = 0x01;

    // MODE2 bits
    static constexpr uint8_t MODE2_OUTDRV  = 0x04; // totem-pole outputs

    TwoWire &_wire;
    uint8_t  _addr;
    uint8_t  _oePin;
    uint32_t _oscHz;
    uint8_t  _prescale;
    bool     _ok;
    TCA9548A *_mux;
    uint8_t   _muxChannel;

    void    _select();

    bool    _write8(uint8_t reg, uint8_t value);
    uint8_t _read8(uint8_t reg);
    void    _write4(uint8_t reg, uint16_t on, uint16_t off);
};

#endif
