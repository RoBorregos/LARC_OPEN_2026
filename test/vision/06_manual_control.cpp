/**
 * @file 06_manual_servo_control.cpp
 * @date 2026-09-15
 * @brief Vision state-machine reference: commands, return values and examples.
 *
 * Commands below return void (no success value). Getters return cached state;
 * they do not ask the camera for a new reading. Keep calling update().
 * All state-condition placeholders are false: connect them to your states.
 */

#include <Arduino.h>
#include "robot/instances/instances.hpp"

// Sketch-only helper: which confirmed benefit is centred in front of us?
// Returns 0 = RED, 1 = BLUE, 255 = none OR the reading is not usable.
// It does not open a door. This is the latest confirmed target, not a distance,
// confidence score or list of everything visible. It may remain cached until
// another command is confirmed or the link watchdog expires.
uint8_t benefitInFront()
{
    if (!vision.isLinkUp() || !vision.inBenefitsPhase() ||
        !vision.isBenefitsRunning() || vision.hasCriticalError() ||
        vision.hasBenefitsError())
        return VisionLink::kNoBenefit;

    return vision.seenBenefit();
}

/* Read-only API reference (none of these calls moves a servo):
 *
 * vision.benefitSeen()       -> bool: a confirmed target is cached.
 * vision.seenBenefit()       -> uint8_t: 0 RED, 1 BLUE, 255 no target.
 * VisionLink::benefitName(n) -> const char*: "RED", "BLUE", or "-".
 * vision.benefitHeld()       -> bool: an explicitly requested door is held.
 * vision.heldBenefit()       -> uint8_t: 0 RED, 1 BLUE, 255 no held door.
 * vision.benefitPending()    -> bool: openBenefit() is waiting for a target.
 * Seen and held are different: the camera can see BLUE while RED is held.
 * Held/pending describe software commands, not physical door feedback.
 *
 * vision.phase()            -> VisionProto::Phase: IDLE=0, BEANS=1,
 *                             BENEFITS=2, HALT=3 (applied, not requested).
 * vision.inBeansPhase()     -> bool: applied phase is BEANS.
 * vision.inBenefitsPhase()  -> bool: applied phase is BENEFITS.
 * vision.isLinkUp()         -> bool: valid traffic has established the link
 *                             and the watchdog has not timed out.
 * vision.isOrinReady()      -> bool: READY bit in the last accepted status.
 * vision.isBeansRunning()   -> bool: BEANS-running bit in that status.
 * vision.isBenefitsRunning()-> bool: BENEFITS-running bit in that status.
 * Status bits do not acknowledge a particular request; check phase too.
 *
 * vision.hasIntakeError()   -> bool: intake-source fault latched.
 * vision.hasCameraError()   -> bool: camera fault latched.
 * vision.hasSeparatorError()-> bool: separator-source fault latched.
 * vision.hasBenefitsError() -> bool: benefits-source fault latched.
 * vision.linkLost()         -> bool: watchdog fault latched.
 * vision.hasCriticalError()-> bool: intake OR camera OR link-lost fault.
 * vision.hasError()         -> bool: any of the above faults.
 * These faults stay set until clearErrors(); clearing is not a repair.
 *
 * vision.lastStatus()       -> uint8_t bitmask: 0x01 intake fault,
 *   0x02 separator fault, 0x04 benefits fault, 0x08 camera fault,
 *   0x10 beans running, 0x20 benefits running, 0x40 Orin ready.
 *   Example: 0x60 = ready + benefits running, with no reported fault bits.
 * vision.confirmRemaining()-> uint8_t: frames still needed for confirmation.
 * vision.lastSeparatorInvalid() -> bool: last accepted separator code was
 *                                 invalid (forced to neutral).
 * vision.stats() -> const VisionProto::LinkStats&: counters named accepted,
 *                   rejected and duplicates.
 *
 * Additional commands (all void):
 * vision.requestStatus(); -> sends 0xA2 every call; response arrives later
 *                           through update(). Use on an event, not every loop.
 * vision.resetGuards();   -> allows startBeans/startBenefits/stop to transmit
 *                           again; no serial output or servo motion itself.
 * vision.handleFaults();  -> if critical: stop(), safeState(), clearErrors().
 *                           Does not stop the drive; explicit block below does
 *                           leave a place for the robot's drive-stop call.
 */

void setup()
{
    Serial.begin(Constants::VisionConfig::kSerialBaud);
    servos.begin(); // Initialize actuators; returns void.
    vision.begin(); // Reset vision state and park actuators; returns void.

    // void: report targets without automatically opening their doors.
    // Default is false (doors follow the stream). Set true before detection.
    // Changing this setting alone does not close an already-open door.
    vision.setBenefitsOnRequest(true);
}

void loop()
{
    // void: receive/validate frames, confirm commands, drive enabled servos,
    // run the link watchdog and service servo timers. Call every loop.
    vision.update();

    // Replace these with your state conditions. Entry/exit/ready/drop events
    // should last one loop; a continuously true ready can reopen after close.
    const bool atBeansCorner    = false;
    const bool atBeansEnd       = false;
    const bool atBenefitsCorner = false;
    const bool atBenefitsEnd    = false;
    const bool ready            = false;
    const bool beanDropped      = false;
    const bool eStop            = false;

    if (eStop || vision.hasCriticalError())
    {
        vision.stop();       // void: send STOP (0xA1), guarded against repeats.
        vision.safeState();  // void: park now, close doors, clear held/pending/
                             // seen target, set local IDLE, reset send guards.
        // LARC.stop();      // Enable when integrating the drive system.
        // Faults remain latched. After deliberate recovery, call
        // vision.clearErrors(); (void) and re-enter the desired phase.
        return;              // Do not start/open anything in this iteration.
    }

    const bool canSort = !vision.hasSeparatorError(); // bool
    const bool canDrop = !vision.hasBenefitsError();  // bool

    if (atBeansEnd || atBenefitsEnd)
    {
        vision.leave();     // void: stop() + resetGuards(); no immediate park.
        vision.safeState(); // Park locally without waiting for the Orin reply.
        return;
    }

    // void: send 0xA0 / 0xA3 once per guard reset. These requests neither
    // change the applied phase immediately nor return camera data.
    if (atBeansCorner && canSort)    vision.startBeans();
    if (atBenefitsCorner && canDrop) vision.startBenefits();

    // Just identify the benefit: startBenefits() on entry, keep update()
    // running, then read this helper. No openBenefit() call is required.
    const uint8_t front = benefitInFront();
    const char *frontName = VisionLink::benefitName(front);
    // Example results:
    // confirmed RED:  front == 0,   frontName == "RED" (text)
    // confirmed BLUE: front == 1,   frontName == "BLUE" (text)
    // none/not usable:front == 255, frontName == "-" (text)
    // Compare front numerically; do not compare C strings with ==.
    // Do not use `if (front)`: RED is zero and no target is 255!
    (void)frontName; // Available to your state machine; not printed on the link.

    if (beanDropped || !canDrop)
        vision.closeBenefit(); // void: cancel pending/held request, close both.
    else if (ready && front != VisionLink::kNoBenefit)
        vision.openBenefit();  // void: hold the currently seen door open.

    // Alternative: openBenefit() without a known target arms a request for
    // the next confirmed BENEFITS target: benefitPending() then returns true.
    // After opening: benefitHeld() == true, heldBenefit() == 0 or 1,
    // benefitPending() == false. In this setup it stays open until closed;
    // safe state, phase changes, or watchdog can also close it. A ServoSystem
    // constructed with timedBenefits=true can close it on its timer too.
    // closeBenefit() does not erase seenBenefit(): detection and door differ.

    // servos.safeState() is also void: parks actuators only, without resetting
    // vision's phase or telling the Orin. Later update() can drive them again.

    // void: debug text, automatically suppressed when output is the Orin port.
    // Current instances use Serial for Orin, so this produces NO console text.
    // With a separate debug port, an example printState line is:
    // link=UP orin=READY beans=0 benefits=1 up=0 lo=0 sep=0 door=RED err=----
    // beans/benefits = applied phase; up/lo = intake deployment (0/1);
    // sep = 0 neutral, 1 left, 2 right; door = held RED/BLUE, WAITING or -;
    // err = latched intake/camera/separator/benefits flags (I/C/S/B or -).
    // This line does NOT show seenBenefit(); door is the held/pending request.
    vision.printState(Serial);
}
