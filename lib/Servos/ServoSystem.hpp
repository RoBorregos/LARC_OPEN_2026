/**
   @file ServoSystem.hpp
   @date 2026-08-19
 
   @brief The robot's five servos behind one object, driven by a single
    PCA9685 16-channel PWM board over I2C. This is a PURE ACTUATOR LAYER. It knows angles, 
    travel limits and the benefit door timing, nothing else. 
  
    Safety
    Every commanded angle is clamped into that servo's mechanical limits
    safeState() is the single definition of "everything harmless": intakes
    home, separator neutral, both doors closed and re armed.
 **/

#ifndef SERVO_SYSTEM_HPP
#define SERVO_SYSTEM_HPP

#include <Arduino.h>
#include "PCA9685.hpp"
#include "TCA9548A/TCA9548A.h"

#include "constants.h"
#include "pins.h"

class ServoSystem
{
public:
    // Which bin LEFT and RIGHT physically mean (mature vs immature) is the
    // caller's business; this library only knows positions.
    enum class SeparatorPos : uint8_t
    {
        NEUTRAL = 0,
        LEFT    = 1,
        RIGHT   = 2
    };

    enum class BenefitPhase : uint8_t
    {
        ARMED      = 0, // closed, ready to open
        OPEN       = 1, // open, auto closes after kBenefitOpenMs
        WAIT_REARM = 2  // closed, needs an explicit close before it can open again
    };

    // timedBenefits true closes an open door by itself after
    // kBenefitOpenMs. False holds it until setBenefit(which, false).
    //
    // i2cMux: the robot's shared TCA9548A (instances.cpp passes &i2cMux).
    // Leave it nullptr in standalone test sketches: ServoSystem then uses
    // its own TCA9548A on Pins::I2cMux. Only matters when
    // Pins::Servos::kTcaChannel != 255.
    explicit ServoSystem(bool timedBenefits = false, TCA9548A *i2cMux = nullptr);

    // Lifecycle
    void begin();
    bool driverOk() const { return _pwm.ok(); } // PCA9685 answered in begin()
    void update(); // uses millis()
    void update(uint32_t nowMs); // rollover safe, does nothing untimed

    // Commands (immediate, idempotent)
    void setIntakeUpper(bool deployed);
    void setIntakeLower(bool deployed);
    void setSeparator(SeparatorPos pos);

    // which: 0 = benefit 1, 1 = benefit 2.
    void setBenefit(uint8_t which, bool openRequest);

    // Shut and re arm both doors. Idempotent; safe to call every loop.
    void closeBenefits();

    // Everything harmless, immediately. The one definition of "safe".
    void safeState();

    // Status
    bool         intakeUpperDeployed() const { return _intakeUpper; }
    bool         intakeLowerDeployed() const { return _intakeLower; }
    SeparatorPos separatorPos()        const { return _separator; }
    BenefitPhase benefitPhase(uint8_t which) const;
    bool         timedBenefits() const { return _timedBenefits; }

    // ms left in the current open window; 0 when untimed or not open.
    uint32_t benefitOpenRemainingMs(uint8_t which, uint32_t nowMs) const;

    // Last angle actually written, by Constants::ServoConfig::ServoIndex.
    // 255 = never written / bad index. For tests and debug.
    uint8_t lastAngle(uint8_t servoIndex) const;

private:
    struct BenefitState
    {
        BenefitPhase phase;
        uint32_t     openedAtMs;
    };

    TCA9548A _ownMux; // fallback when no shared mux is given
    PCA9685  _pwm;

    bool         _intakeUpper;
    bool         _intakeLower;
    SeparatorPos _separator;
    BenefitState _benefit[2];
    uint8_t      _lastAngle[Constants::ServoConfig::SERVO_COUNT];
    bool         _timedBenefits;

    // Skips the I2C write when the angle is already applied.
    void _writeAngle(uint8_t servoIndex, uint8_t angleDeg);
};

#endif
