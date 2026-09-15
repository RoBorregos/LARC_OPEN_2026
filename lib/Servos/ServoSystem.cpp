/**
  @file ServoSystem.cpp
  @date 2026-08-19
 
  @brief Implementation of the five servo PCA9685 actuator layer.
 */

#include "ServoSystem.hpp"

using namespace Constants::ServoConfig;
static constexpr uint8_t kBenefitIndex[2]  = { BENEFIT_1,       BENEFIT_2       };
static constexpr uint8_t kBenefitClosed[2] = { kBenefit1Closed, kBenefit2Closed };
static constexpr uint8_t kBenefitOpen[2]   = { kBenefit1Open,   kBenefit2Open   };

// Lifecycle

ServoSystem::ServoSystem(bool timedBenefits)
    : _pwm(Pins::Servos::kPcaI2cAddress)
    , _intakeUpper(false)
    , _intakeLower(false)
    , _separator(SeparatorPos::NEUTRAL)
    , _timedBenefits(timedBenefits)
{
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        _lastAngle[i] = 255; // unknown until begin() writes something

    for (uint8_t i = 0; i < 2; i++)
    {
        _benefit[i].phase      = BenefitPhase::ARMED;
        _benefit[i].openedAtMs = 0;
    }
}

void ServoSystem::begin()
{
    _pwm.begin();
    _pwm.setOscillatorFrequency(kPcaOscillatorHz);
    _pwm.setPWMFreq(kServoPwmFreqHz);
    safeState();
}

void ServoSystem::update()
{
    update(millis());
}

void ServoSystem::update(uint32_t nowMs)
{
    if (!_timedBenefits)
        return;

    for (uint8_t i = 0; i < 2; i++)
    {
        BenefitState &b = _benefit[i];
        if (b.phase == BenefitPhase::OPEN && (nowMs - b.openedAtMs) >= kBenefitOpenMs)
        {
            _writeAngle(kBenefitIndex[i], kBenefitClosed[i]);
            b.phase = BenefitPhase::WAIT_REARM;
        }
    }
}

// Commands

void ServoSystem::setIntakeUpper(bool deployed)
{
    _intakeUpper = deployed;
    _writeAngle(INTAKE_UPPER, deployed ? kIntakeUpperDeploy : kIntakeUpperHome);
}

void ServoSystem::setIntakeLower(bool deployed)
{
    _intakeLower = deployed;
    _writeAngle(INTAKE_LOWER, deployed ? kIntakeLowerDeploy : kIntakeLowerHome);
}

void ServoSystem::setSeparator(SeparatorPos pos)
{
    uint8_t angle = kSeparatorNeutral;
    switch (pos)
    {
        case SeparatorPos::LEFT:    angle = kSeparatorLeft;    break;
        case SeparatorPos::RIGHT:   angle = kSeparatorRight;   break;
        case SeparatorPos::NEUTRAL: angle = kSeparatorNeutral; break;
    }
    _separator = pos;
    _writeAngle(SEPARATOR, angle);
}

void ServoSystem::setBenefit(uint8_t which, bool openRequest)
{
    if (which > 1)
        return;

    BenefitState &b = _benefit[which];

    if (openRequest)
    {
        // Only an ARMED door opens. A door in WAIT_REARM ignores a repeated
        // open request, so a command stream stuck on "open" opens once.
        if (b.phase == BenefitPhase::ARMED)
        {
            _writeAngle(kBenefitIndex[which], kBenefitOpen[which]);
            b.openedAtMs = millis();
            b.phase      = BenefitPhase::OPEN;
        }
        return;
    }

    // Close: shut it if it is still open, and re-arm either way.
    if (b.phase != BenefitPhase::ARMED)
    {
        _writeAngle(kBenefitIndex[which], kBenefitClosed[which]);
        b.phase = BenefitPhase::ARMED;
    }
}

void ServoSystem::closeBenefits()
{
    setBenefit(0, false);
    setBenefit(1, false);
}

void ServoSystem::safeState()
{
    setIntakeUpper(false);
    setIntakeLower(false);
    setSeparator(SeparatorPos::NEUTRAL);
    closeBenefits();
}

// Status

ServoSystem::BenefitPhase ServoSystem::benefitPhase(uint8_t which) const
{
    return (which <= 1) ? _benefit[which].phase : BenefitPhase::ARMED;
}

uint32_t ServoSystem::benefitOpenRemainingMs(uint8_t which, uint32_t nowMs) const
{
    if (which > 1 || !_timedBenefits || _benefit[which].phase != BenefitPhase::OPEN)
        return 0;

    const uint32_t elapsed = nowMs - _benefit[which].openedAtMs;
    return (elapsed >= kBenefitOpenMs) ? 0 : (kBenefitOpenMs - elapsed);
}

uint8_t ServoSystem::lastAngle(uint8_t servoIndex) const
{
    return (servoIndex < SERVO_COUNT) ? _lastAngle[servoIndex] : 255;
}

// Output

void ServoSystem::_writeAngle(uint8_t servoIndex, uint8_t angleDeg)
{
    if (servoIndex >= SERVO_COUNT)
        return;

    const ServoCalib &c = kCalib[servoIndex];

    // Mechanical limits win over any commanded angle, always.
    if (angleDeg < c.minAngleDeg) angleDeg = c.minAngleDeg;
    if (angleDeg > c.maxAngleDeg) angleDeg = c.maxAngleDeg;

    if (angleDeg == _lastAngle[servoIndex])
        return; 
    const uint16_t us = (uint16_t)(c.minPulseUs +
        ((uint32_t)(c.maxPulseUs - c.minPulseUs) * angleDeg) / 180U);

    _pwm.writeMicroseconds(c.channel, us);
    _lastAngle[servoIndex] = angleDeg;
}
