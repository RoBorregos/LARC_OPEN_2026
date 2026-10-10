/**
   @file VisionLink.cpp
   @date 2026-08-19
  
   @brief Implementation of the Orin link, including the confirmation filter and the safety watchdog.
 */

#include "VisionLink.hpp"

using VisionProto::Phase;
using VisionProto::SeparatorCode;

namespace
{
// The confirmation depth, read once from the one place it is configured.
constexpr uint8_t kConfirmFrames = Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES;

ServoSystem::SeparatorPos toServoPos(SeparatorCode code)
{
    switch (code)
    {
        case SeparatorCode::LEFT:  return ServoSystem::SeparatorPos::LEFT;
        case SeparatorCode::RIGHT: return ServoSystem::SeparatorPos::RIGHT;
        default:                   return ServoSystem::SeparatorPos::NEUTRAL;
    }
}
} // namespace

// Lifecycle

VisionLink::VisionLink(Stream &port, ServoSystem &servos, uint32_t timeoutMs)
    : _serial(port)
    , _servos(servos)
    , _link(timeoutMs)
    , _appliedPhase(Phase::IDLE)
    , _status(0)
    , _separatorInvalid(false)
    , _intakeError(false)
    , _cameraError(false)
    , _separatorError(false)
    , _benefitsError(false)
    , _linkLost(false)
    , _beansSent(false)
    , _benefitsSent(false)
    , _stopSent(false)
    , _openRequest(false)
    , _heldBenefit(kNoBenefit)
    , _seenBenefit(kNoBenefit)
    , _boxVisible(false)
    , _benefitsOnRequest(false)
{
    _confirm.reset((uint8_t)Phase::IDLE, 0);
}

void VisionLink::begin()
{
    _status           = 0;
    _separatorInvalid = false;
    clearErrors();
    resetGuards();
    _link.resetStats();
    _goSafe(); // includes closing and re arming both benefit doors
}

void VisionLink::update()
{
    const uint32_t now = millis();

    // Consume everything that arrived since the last loop
    while (_serial.available())
    {
        const VisionProto::LinkEvent ev =
            _link.feedByte((uint8_t)_serial.read(), now);

        // Rejected and duplicate frames fall through here on purpose: they
        // give no confirmation credit and do not feed the watchdog.
        if (!ev.accepted)
            continue;

        _status           = ev.cmd.status;
        _separatorInvalid = ev.cmd.separatorInvalid;
        _latchFaults(ev.cmd.status);

        // Safety first, and without waiting for confirmation.
        _applySafetyImmediate(ev.cmd);

        // Everything else has to earn REQUIRED_CONFIRMATION_FRAMES.
        if (_confirm.sample((uint8_t)ev.cmd.phase, ev.cmd.rawPayload, kConfirmFrames))
            _applyCommand(ev.cmd);
    }

    // Watchdog: the Orin went quiet
    if (_link.checkTimeout(now))
    {
        _goSafe();
        _status = 0;
        // A dead link is a critical fault: the state machine must see that
        // nobody is steering the sorter any more.
        _linkLost = true;
        // Let the state machine re-request its phase once the Orin is back.
        resetGuards();
    }

    _servos.update(now);

    // Only reachable with a timed ServoSystem: the door shut itself, so
    // re-arm it and let go of the hold.
    if (_heldBenefit != kNoBenefit &&
        _servos.benefitPhase(_heldBenefit) != ServoSystem::BenefitPhase::OPEN)
    {
        _servos.setBenefit(_heldBenefit, false);
        _heldBenefit = kNoBenefit;
    }
}

// Phase requests

void VisionLink::startBeans()
{
    if (!_beansSent)
    {
        _serial.write(VisionProto::kCmdStartBeans);
        _beansSent = true;
    }
}

void VisionLink::startBenefits()
{
    if (!_benefitsSent)
    {
        _serial.write(VisionProto::kCmdStartBenefits);
        _benefitsSent = true;
    }
}

void VisionLink::stop()
{
    if (!_stopSent)
    {
        _serial.write(VisionProto::kCmdStop);
        _stopSent = true;
    }
}

void VisionLink::requestStatus()
{
    _serial.write(VisionProto::kCmdStatus);
}

void VisionLink::leave()
{
    stop();
    resetGuards();
}

void VisionLink::openBenefit()
{
    if (_heldBenefit != kNoBenefit)
        return;

    if (_appliedPhase == Phase::BENEFITS && _seenBenefit != kNoBenefit)
    {
        _heldBenefit = _seenBenefit;
        _openRequest = false;
        _servos.setBenefit(_heldBenefit, true);
        return;
    }

    _openRequest = true;
}

void VisionLink::closeBenefit()
{
    _dropHold();
    _servos.closeBenefits();
}

void VisionLink::resetGuards()
{
    _beansSent    = false;
    _benefitsSent = false;
    _stopSent     = false;
}

// Status

bool VisionLink::isOrinReady() const
{
    return (_status & VisionProto::kStatusOrinReady) != 0;
}

bool VisionLink::isBeansRunning() const
{
    return (_status & VisionProto::kStatusBeansRunning) != 0;
}

bool VisionLink::isBenefitsRunning() const
{
    return (_status & VisionProto::kStatusBenefitsRunning) != 0;
}

void VisionLink::clearErrors()
{
    _intakeError    = false;
    _cameraError    = false;
    _separatorError = false;
    _benefitsError  = false;
    _linkLost       = false;
}

void VisionLink::safeState()
{
    _goSafe();
    resetGuards();
}

void VisionLink::handleFaults()
{
    if (!hasCriticalError())
        return;

    stop();
    safeState();
    clearErrors();
}

const char *VisionLink::benefitName(uint8_t which)
{
    if (which == 0) return "BLUE";
    if (which == 1) return "RED";
    return "-";
}

void VisionLink::printState(Stream &out) const
{
    if (&out == &_serial)
        return;

    char err[5] = "----"; // intake, camera, separator, benefits
    if (_intakeError)    err[0] = 'I';
    if (_cameraError)    err[1] = 'C';
    if (_separatorError) err[2] = 'S';
    if (_benefitsError)  err[3] = 'B';

    out.printf("link=%s orin=%s beans=%d benefits=%d up=%d lo=%d sep=%d door=%s err=%s\n",
               isLinkUp() ? "UP" : "DOWN",
               isOrinReady() ? "READY" : "-",
               inBeansPhase(),
               inBenefitsPhase(),
               _servos.intakeUpperDeployed(),
               _servos.intakeLowerDeployed(),
               (int)_servos.separatorPos(),   // LEFT = mature, RIGHT = overmature
               _openRequest ? "WAITING" : benefitName(_heldBenefit),
               err);
}

void VisionLink::_latchFaults(uint8_t status)
{
    if (status & VisionProto::kStatusMainFault)      _intakeError    = true;
    if (status & VisionProto::kStatusCameraFault)    _cameraError    = true;
    if (status & VisionProto::kStatusSeparatorFault) _separatorError = true;
    if (status & VisionProto::kStatusBenefitsFault)  _benefitsError  = true;
}

// Applying commands

void VisionLink::_applySafetyImmediate(const VisionProto::Command &cmd)
{
    // HALT and IDLE mean "stop now". Neither waits for a second opinion.
    if (cmd.phase == Phase::HALT || cmd.phase == Phase::IDLE)
    {
        if (_appliedPhase != cmd.phase)
        {
            _servos.safeState();
            _dropHold();
            _seenBenefit = kNoBenefit;
            _boxVisible  = false;
            _appliedPhase = cmd.phase;
            // The payload of these phases is always 0 (validated upstream).
            _confirm.reset((uint8_t)cmd.phase, 0);
        }
        return;
    }

    if (_appliedPhase == Phase::BENEFITS && cmd.phase != Phase::BENEFITS)
    {
        _servos.closeBenefits();
        _dropHold();
        _seenBenefit = kNoBenefit;
        _boxVisible  = false;
    }

    // A held door ignores the stream; only closeBenefit() shuts it.
    if (cmd.phase == Phase::BENEFITS && _heldBenefit == kNoBenefit)
    {
        if (!cmd.benefit1Open) _servos.setBenefit(0, false);
        if (!cmd.benefit2Open) _servos.setBenefit(1, false);
    }
}

void VisionLink::_applyCommand(const VisionProto::Command &cmd)
{
    _appliedPhase = cmd.phase;

    switch (cmd.phase)
    {
        case Phase::BEANS:
            _servos.closeBenefits();
            _servos.setIntakeUpper(cmd.intakeUpper);
            _servos.setIntakeLower(cmd.intakeLower);
            _servos.setSeparator(toServoPos(cmd.separator));
            break;

        case Phase::BENEFITS:
            _servos.setIntakeUpper(false);
            _servos.setIntakeLower(false);
            _servos.setSeparator(ServoSystem::SeparatorPos::NEUTRAL);
            _applyBenefits(cmd);
            break;

        case Phase::IDLE:
        case Phase::HALT:
            _servos.safeState();
            break;
    }
}

void VisionLink::_applyBenefits(const VisionProto::Command &cmd)
{
    if (cmd.benefit1Open)      _seenBenefit = 0;
    else if (cmd.benefit2Open) _seenBenefit = 1;
    else                       _seenBenefit = kNoBenefit;
    _boxVisible = cmd.boxVisible || _seenBenefit != kNoBenefit;

    if (_heldBenefit != kNoBenefit)
        return;

    // Nobody asked for a door: the frame drives them as it always did.
    if (!_openRequest)
    {
        if (_benefitsOnRequest)
            return;

        _servos.setBenefit(0, cmd.benefit1Open);
        _servos.setBenefit(1, cmd.benefit2Open);
        return;
    }

    if (cmd.benefit1Open)      _heldBenefit = 0;
    else if (cmd.benefit2Open) _heldBenefit = 1;
    else                       return; // the Orin has not decided yet

    _servos.setBenefit(_heldBenefit, true);
    _openRequest = false;
}

void VisionLink::_dropHold()
{
    _openRequest = false;
    _heldBenefit = kNoBenefit;
}

void VisionLink::_goSafe()
{
    _servos.safeState();
    _dropHold();
    _seenBenefit = kNoBenefit;
    _boxVisible  = false;
    _appliedPhase = Phase::IDLE;
    _confirm.reset((uint8_t)Phase::IDLE, 0);
}
