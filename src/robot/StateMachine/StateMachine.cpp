#include <Arduino.h>
#include "StateMachine.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "Vision.hpp"
#include "testOdometry.hpp"

namespace
{
    static constexpr float kObstacleDistanceCm = 20.0f;

    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START:
            return F(""); // F("START ♡ ♡ ♡");
        case STATES::POOL:
            return F(""); // F("POOL");
        case STATES::LOOKFORLINE:
            return F(""); // F("LOOKFORLINE");
        case STATES::LOOKFORCORNER:
            return F(""); // F("LOOKFORCORNER");
        case STATES::BEANS:
            return F(""); // F("BEANS");
        case STATES::BEANSGOBACK:
            return F(""); // F("BEANSGOBACK");
        case STATES::POOLSGOBACK:
            return F(""); // F("POOLSGOBACK");
        case STATES::LOOKFORLINEBACKWARDS:
            return F(""); // F("LOOKFORLINEBACKWARDS");
        case STATES::BENEFITSSTARTCORNER:
            return F(""); // F("BENEFITSSTARTCORNER");
        case STATES::BENEFITS:
            return F(""); // F("BENEFITS");
        case STATES::STOP:
            return F(""); // F("STOP ♡ ♡ ♡ ♡ ♡");
        default:
            return F(""); // F("DEFAULT");
        }
    }
}

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::START; // always in START

    state_start_time = millis();

    visionLeft = 0;
    visionRight = 0;

    // Elevator

    pinMode(limitSwitch, INPUT_PULLUP);

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

    const int linePos = qtrFront.getPosition(); // Para el PID → más suave
    const bool onLine = qtrFront.onLine();
    const float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    const float vx = -lineCorr;

    const bool FL = ir.getState(IRLine::FL);
    const bool FR = ir.getState(IRLine::FR);
    const bool BL = ir.getState(IRLine::BL);
    const bool BR = ir.getState(IRLine::BR);

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

    const bool frontLeftDetectedLine = FL; // Also used for corner
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BL;
    const bool backRightDetectedLine = BR;
    const bool frontDetectedLine = (FL || FR); // Hacer que con el qtr tambien detecte linea
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    // DESPUÉS
    static constexpr uint32_t kTofWarmupMs = 500;
    static uint32_t tofReadyTimestamp = 0;
    if (tofReadyTimestamp == 0 && (tofLeft.isValid() || tofRight.isValid()))
        tofReadyTimestamp = now;
    const bool tofReady = tofReadyTimestamp != 0 &&
                        (now - tofReadyTimestamp) > kTofWarmupMs;

    const bool obstacleLeftNow  = tofReady
                            && tofLeft.isValid()
                            && tofLeft.getDistanceCm()  < kObstacleDistanceCm;

    const bool obstacleRightNow = tofReady
                            && tofRight.isValid()
                            && tofRight.getDistanceCm() < kObstacleDistanceCm;

    static bool obstacleLatched = false;
    static uint32_t obstacleClearStartMs  = 0;
    static uint32_t obstacleDetectStartMs = 0;
    static constexpr uint32_t kObstacleReleaseMs  = 400;
    static constexpr uint32_t kObstacleConfirmMs  = 0;

    if (!obstacleLatched)
    {
    if (obstacleLeftNow || obstacleRightNow)
    {
    if (obstacleDetectStartMs == 0)
        obstacleDetectStartMs = now;

    if ((now - obstacleDetectStartMs) >= kObstacleConfirmMs)
    {
        obstacleLatched       = true;
        obstacleClearStartMs  = 0;
        obstacleDetectStartMs = 0;
    }
    }
    else
    {
    obstacleDetectStartMs = 0; // reset si deja de verse
    }
    }
    else
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleClearStartMs = 0;
        }
        else
        {
            if (obstacleClearStartMs == 0)
                obstacleClearStartMs = now;

            if ((now - obstacleClearStartMs) >= kObstacleReleaseMs)
            {
                obstacleLatched = false;
                obstacleClearStartMs = 0;
            }
        }
    }
    const bool obstacle = obstacleLatched;

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

    Serial.println(mainStateName(currentState));
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
