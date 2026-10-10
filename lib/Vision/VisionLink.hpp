/**
   @file VisionLink.hpp
   @date 2026-08-19
  
   @brief The whole Orin link, behind a handful of verbs.
  
    This is the ONLY class the state machine talks to about vision. It owns
    every piece of the protocol so nothing else has to know one exists:
  
    reading the serial port
    framing, CRC-8, version and phase validation, reserved-bit checks
    sequence numbers and duplicate rejection
    the single confirmation filter (CommandConfirm)
    the link watchdog and the safe state it forces
    phase gating: which actuators a stage may address at all
    driving ServoSystem
    the four uplink request bytes

    Typical use:
    ServoSystem servos;
    VisionLink  vision(Serial, servos);   // USB link to the Orin
  
    void setup() { servos.begin(); vision.begin(); }
    void loop()  { vision.update(); } // drives the servos for you
  
    update() must be called every loop. just consumes
    whatever bytes have arrived, applies whatever became confirmed, runs
    the watchdog, and services the benefit door timers.
 */

#ifndef VISION_LINK_HPP
#define VISION_LINK_HPP

#include <Arduino.h>

#include "constants.h"
#include "CommandConfirm.hpp"
#include "VisionProtocol.hpp"
#include "ServoSystem.hpp"

class VisionLink
{
public:
    static_assert(Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES >= 1 &&
                  Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES <= 3,
                  "Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES must be 1, 2 or 3");

    VisionLink(Stream &port, ServoSystem &servos,
               uint32_t timeoutMs = Constants::VisionConfig::kLinkTimeoutMs);

    // put the actuators in the
    // safe state with both benefit doors closed.
    void begin();

    /// Call every loop. Consumes serial, applies confirmed commands, runs
    /// the watchdog, services the benefit timers. Never blocks.
    void update();

    // Phase requests (one byte out, guarded against repeats)
    void startBeans();
    void startBenefits();
    void stop();
    void requestStatus();

    // stop() plus resetGuards(), the pair every phase exit needs.
    void leave();

    // Benefit doors
    // openBenefit() opens the door of the box being seen right now, or arms
    // the request so the next confirmed BENEFITS frame decides. The door
    // stays open until closeBenefit(). While one is held the Orin cannot
    // pick another.
    void openBenefit();
    void closeBenefit();

    // true: the stream only reports the box (benefitSeen()), a door opens
    // only through openBenefit(). false: the doors follow the stream.
    void setBenefitsOnRequest(bool on) { _benefitsOnRequest = on; }

    static constexpr uint8_t kNoBenefit = 255;

    bool    benefitHeld()    const { return _heldBenefit != kNoBenefit; }
    bool    benefitPending() const { return _openRequest; }
    uint8_t heldBenefit()    const { return _heldBenefit; } // 0 or 1, else kNoBenefit

    // The door the Orin is asking for right now (box centred in the camera).
    bool    benefitSeen()    const { return _seenBenefit != kNoBenefit; }
    uint8_t seenBenefit()    const { return _seenBenefit; } // 0 or 1, else kNoBenefit

    // Any part of a box in view (centred or not). False once it has fully
    // left the camera, which is what a stop should re-arm on.
    bool    benefitVisible() const { return _boxVisible; }

    // Which box a door belongs to. Mirrors BOX_DOOR in dispatcher.py.
    static const char *benefitName(uint8_t which);

    // Re arm the request guards so the next start*/stop() transmits again.
    void resetGuards();

    // Status 

    // The phase the Teensy is currently acting on (not the requested one).
    VisionProto::Phase phase() const { return _appliedPhase; }
    bool inBeansPhase()        const { return _appliedPhase == VisionProto::Phase::BEANS; }
    bool inBenefitsPhase()     const { return _appliedPhase == VisionProto::Phase::BENEFITS; }

    // False once the watchdog has fired, true again on the next valid frame.
    bool isLinkUp() const { return _link.linkUp(); }

    // Live flags from the STATUS byte of the last accepted frame.
    bool isOrinReady()         const;
    bool isBeansRunning()    const;
    bool isBenefitsRunning() const;

    // They stay set until clearErrors().
    bool hasIntakeError()    const { return _intakeError; }   // the Orin's intake source died
    bool hasCameraError()    const { return _cameraError; }
    bool hasSeparatorError() const { return _separatorError; }
    bool hasBenefitsError()  const { return _benefitsError; }
    bool linkLost()          const { return _linkLost; }      // the watchdog fired

    // The three reasons nobody is steering the sorter any more. Stop the
    // robot on this one.
    bool hasCriticalError() const { return _intakeError || _cameraError || _linkLost; }
    bool hasError() const { return hasCriticalError() || _separatorError || _benefitsError; }

    uint8_t lastStatus() const { return _status; }
    void clearErrors();

    // Park and clear once a critical fault latches. Call every loop, or
    // write the block yourself when the drive has to stop with it.
    void handleFaults();

    // Everything harmless now: servos safe, doors shut, phase IDLE and the
    // guards re-armed so the next start*() transmits. Says nothing to the
    // Orin; pair it with stop() when the Orin should quit too.
    void safeState();

    // Debug

    // One line of everything this object and the servos are doing.
    void printState(Stream &out) const;

    const VisionProto::LinkStats &stats() const { return _link.stats(); }

    uint8_t confirmRemaining() const
    {
        return _confirm.remaining(Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES);
    }
    bool lastSeparatorInvalid() const { return _separatorInvalid; }

private:
    Stream                   &_serial;
    ServoSystem              &_servos;
    VisionProto::LinkManager  _link;
    CommandConfirm            _confirm;

    VisionProto::Phase _appliedPhase;
    uint8_t            _status;
    bool               _separatorInvalid;

    bool _intakeError;
    bool _cameraError;
    bool _separatorError;
    bool _benefitsError;
    bool _linkLost;

    bool _beansSent;
    bool _benefitsSent;
    bool _stopSent;

    bool    _openRequest;
    uint8_t _heldBenefit;
    uint8_t _seenBenefit;
    bool    _boxVisible;
    bool    _benefitsOnRequest;

    void _applySafetyImmediate(const VisionProto::Command &cmd);

    void _applyCommand(const VisionProto::Command &cmd);

    void _applyBenefits(const VisionProto::Command &cmd);

    // Forget the held door without moving anything.
    void _dropHold();

    // Everything safe, filters reset, phase back to IDLE.
    void _goSafe();

    void _latchFaults(uint8_t status);
};

#endif
