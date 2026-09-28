#include <Arduino.h>
#include "StateMachine.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "testOdometry.hpp"

// ---------------------------------------------------------------------------
// SM_DEBUG -- telemetry over USB Serial.
//
// The Orin vision link runs on this SAME USB port (see instances.cpp:
// "VisionLink vision(Serial, servos)"). Every byte printed here lands in the
// Orin's frame parser, which rejects it as garbage and bumps the reject
// counter. So telemetry is OFF by default and MUST stay off whenever the Orin
// is actually plugged in.
//
// To turn it back on for bench debugging (no Orin attached, or driving the
// link by hand with test/vision/keyboard_command_test.py), either flip the 0
// below to 1, or -- better, no file edit -- add the flag to the env in
// platformio.ini:
//
//     build_flags = ${env:teensy41.build_flags} -D SM_DEBUG=1
//
// ---------------------------------------------------------------------------
#ifndef SM_DEBUG
#define SM_DEBUG 0
#endif

#if SM_DEBUG
namespace
{
    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START:                return F("START");
        case STATES::POOL:                 return F("POOL");
        case STATES::LOOKFORLINE:          return F("LOOKFORLINE");
        case STATES::LOOKFORCORNER:        return F("LOOKFORCORNER");
        case STATES::BEANS:                return F("BEANS");
        case STATES::BEANSGOBACK:          return F("BEANSGOBACK");
        case STATES::POOLSGOBACK:          return F("POOLSGOBACK");
        case STATES::LOOKFORLINEBACKWARDS: return F("LOOKFORLINEBACKWARDS");
        case STATES::BENEFITSSTARTCORNER:  return F("BENEFITSSTARTCORNER");
        case STATES::BENEFITS:             return F("BENEFITS");
        case STATES::STOP:                 return F("STOP");
        default:                           return F("DEFAULT");
        }
    }
}
#endif // SM_DEBUG

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::START; // always in START

    state_start_time = millis();

    // Elevator

    pinMode(limitSwitch, INPUT_PULLUP);

    servos.begin();
    vision.begin();
    vision.requestStatus();

    Wire.begin();
    Wire.setClock(400000);
    i2cMux.begin();

    bool okL = tofLeft.begin();
    bool okR = tofRight.begin();

    Serial.print("tofLeft init: ");  Serial.println(okL ? "OK" : "FAIL");
    Serial.print("tofRight init: "); Serial.println(okR ? "OK" : "FAIL");

    //QTR
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);   // FRONT qtr

    tofLeft.setMaxRange(600);
    tofRight.setMaxRange(600);

    tofLeft.setUpdateInterval(30);
    tofRight.setUpdateInterval(30);

    ir.begin();
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    odomMove_.begin();
    odomMove_.setCommandTimeout(100);
    odomMove_.resetPose();
    odomMove_.captureCurrentYawTarget();

    startState_.begin();
}

void LARCStateMachine::update()
{
    ir.update();
    qtrFront.update();
    vision.update();
    const uint32_t now = millis();
    startStateTime();

    perception_.update(now);
    const PerceptionSnapshot &p = perception_.get();

    const bool onLine = p.onLine;
    const float vx = p.vx;

    const bool FL = p.FL;
    const bool FR = p.FR;
    const bool BL = p.BL;
    const bool BR = p.BR;

#if SM_DEBUG
    static uint32_t debugPrintMs = 0;
    if ((now - debugPrintMs) >= 100)
    {
    debugPrintMs = now;

    // Odometría
    Serial.print(F(" ❤ Odometria❤ | X:"));    Serial.print(odomMove_.getX(),   3);
    Serial.print(F(" Y:"));      Serial.print(odomMove_.getY(),   3);
    Serial.print(F(" Yaw:"));    Serial.print(odomMove_.getThetaDeg(), 1);
    Serial.print(F(" UL:"));     Serial.print(odomMove_.getRpmUL(), 0);
    Serial.print(F(" UR:"));     Serial.print(odomMove_.getRpmUR(), 0);
    Serial.print(F(" LL:"));     Serial.print(odomMove_.getRpmLL(), 0);
    Serial.print(F(" LR:"));     Serial.print(odomMove_.getRpmLR(), 0);

    // Estado actual
    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState); Serial.print(")");

    // ToF
    Serial.print(F(" ❤ Tof❤ |"));
    Serial.print(F(" TL:")); Serial.print(tofLeft.getDistanceCm(), 0);
    Serial.print(F("cm vL:")); Serial.print(tofLeft.isValid() ? "OK" : "NO");
    Serial.print(F(" TR:")); Serial.print(tofRight.getDistanceCm(), 0);
    Serial.print(F("cm vR:")); Serial.print(tofRight.isValid() ? "OK" : "NO");

    // IR
    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(FL);
    Serial.print(F(" FR:"));   Serial.print(FR);
    Serial.print(F(" BL:"));   Serial.print(BL);
    Serial.print(F(" BR:"));   Serial.print(BR);

    // Línea
    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(onLine);
    Serial.print(F(" lPos:"));  Serial.print(qtrFront.getPosition());
    Serial.print(F(" vx:")); Serial.print(vx);
    Serial.println();
    }
#endif // SM_DEBUG

    const bool frontLeftDetectedLine = FL; // Also used for corner
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BL;
    const bool backRightDetectedLine = BR;
    const bool frontDetectedLine = p.frontDetectedLine; // Hacer que con el qtr tambien detecte linea
    const bool backDetected = p.backDetected;
    const bool leftDetectedPool = p.leftDetectedPool;
    const bool rightDetectedPool = p.rightDetectedPool;
    const bool obstacle = p.obstacle;

    switch (currentState)
    {
    case STATES::START:
    {
        bool transitionToPool = false;
        startState_.update(now, transitionToPool);
        if (transitionToPool)
            setState(STATES::POOL);
        break;
    }

    case STATES::POOL:
    {
        bool transitionToLookForLine = false;
        poolState_.update(now, obstacle, leftDetectedPool, rightDetectedPool, transitionToLookForLine);
        if (transitionToLookForLine)
            setState(STATES::LOOKFORLINE);
        break;
    }

    case STATES::LOOKFORLINE:
    {
        bool transitionToCorner = false;
        lookForLineState_.update(now, frontDetectedLine, frontLeftDetectedLine, frontRightDetectedLine, onLine, transitionToCorner);
        if (transitionToCorner)
            setState(STATES::LOOKFORCORNER);
        break;
    }

    case STATES::LOOKFORCORNER:
    {
        bool transitionToBeans = false;
        lookForCornerState_.update(now, backLeftDetectedLine, vx, transitionToBeans);
        if (transitionToBeans)
            setState(STATES::BEANS);
        break;
    }

    case STATES::BEANS:
    {
        bool transitionToBeansGoBack = false;
        bool transitionToPoolsGoBack = false;
        bool transitionToStop = false;
        beansState_.update(now, backRightDetectedLine, onLine, vx, transitionToBeansGoBack, transitionToPoolsGoBack, transitionToStop);
        if (transitionToStop)
            setState(STATES::STOP);
        else if (transitionToBeansGoBack)
            setState(STATES::BEANSGOBACK);
        else if (transitionToPoolsGoBack)
            setState(STATES::POOLSGOBACK);
        break;
    }

    case STATES::BEANSGOBACK:
    {
        bool transitionToBeans = false;
        beansGoBackState_.update(now, frontLeftDetectedLine, onLine, vx, transitionToBeans);
        if (transitionToBeans)
            setState(STATES::BEANS);
        break;
    }

    case STATES::POOLSGOBACK:
    {
        bool transitionToLookForLineBackwards = false;
        poolsGoBackState_.update(now, obstacle, leftDetectedPool, rightDetectedPool, transitionToLookForLineBackwards);
        if (transitionToLookForLineBackwards)
            setState(STATES::LOOKFORLINEBACKWARDS);
        break;
    }

    case STATES::LOOKFORLINEBACKWARDS:
    {
        bool transitionToBenefitsStartCorner = false;
        lookForLineBackwardsState_.update(now, backDetected, backLeftDetectedLine, backRightDetectedLine, transitionToBenefitsStartCorner);
        if (transitionToBenefitsStartCorner)
            setState(STATES::BENEFITSSTARTCORNER);
        break;
    }

    case STATES::BENEFITSSTARTCORNER:
    {
        bool transitionToBenefits = false;
        benefitsStartCornerState_.update(now, frontLeftDetectedLine, vx, onLine, transitionToBenefits);
        if (transitionToBenefits)
            setState(STATES::BENEFITS);
        break;
    }

    case STATES::BENEFITS:
    {
        bool transitionToStop = false;
        benefitsState_.update(now, backLeftDetectedLine, vx, onLine, transitionToStop);
        if (transitionToStop)
            setState(STATES::STOP);
        break;
    }

    case STATES::STOP:
        stopState_.update(now);
        break;

    default:
        stopState_.update(now);
        break;
    }

    // Park the actuators if the link died or a camera/intake fault latched.
    // This runs AFTER the dispatch on purpose: handleFaults() clears the
    // error flags, so the states (BeansState) must get to read
    // vision.hasCriticalError() first.
    vision.handleFaults();
}

void LARCStateMachine::setState(STATES newState)
{
    if (currentState == newState)
        return;

    currentState = newState;
    state_start_time = millis();

    switch (newState)
    {
    case STATES::START:
        startState_.begin();
        break;
    case STATES::POOL:
        poolState_.begin();
        break;
    case STATES::LOOKFORLINE:
        lookForLineState_.begin();
        break;
    case STATES::LOOKFORCORNER:
        lookForCornerState_.begin();
        break;
    case STATES::BEANS:
        beansState_.begin();
        break;
    case STATES::BEANSGOBACK:
        beansGoBackState_.begin();
        break;
    case STATES::POOLSGOBACK:
        poolsGoBackState_.begin();
        break;
    case STATES::LOOKFORLINEBACKWARDS:
        lookForLineBackwardsState_.begin();
        break;
    case STATES::BENEFITSSTARTCORNER:
        benefitsStartCornerState_.begin();
        break;
    case STATES::BENEFITS:
        benefitsState_.begin();
        break;
    case STATES::STOP:
        stopState_.begin();
        break;
    }

    vision.resetGuards();

#if SM_DEBUG
    Serial.print(F("[SM] -> "));
    Serial.println(mainStateName(currentState));
#endif
}

void LARCStateMachine::forceState(STATES newState)
{
    setState(newState);
}

void LARCStateMachine::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::updateControl()
{
    odomMove_.update();
}
