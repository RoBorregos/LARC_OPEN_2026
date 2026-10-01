#include <Arduino.h>
#include "StateMachine.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "Vision.hpp"
#include "testOdometry.hpp"

namespace
{
    static constexpr float kVelocity = 0.48f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs = 4500;
    static constexpr uint32_t kClearDelayMs = 1500;
    static constexpr uint32_t kNoObstacleToCornerMs = 3000;
    static constexpr uint32_t kCornerDeployWazitMs = 1800;

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTofTargetMm = 120;
    static constexpr uint16_t kTofHardStopMm = 105;

    static constexpr float kTofMinSpeed = 0.10f;
    static constexpr float kTofMaxSpeed = 0.35f;
    static constexpr float kObstacleDistanceCm = 20.0f;

    static constexpr float kDistKp = 0.0012f;
    static constexpr float kDistKi = 0.0f;
    static constexpr float kDistKd = 0.00015f;

    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START:
            return F("");
        case STATES::POOL:
            return F("");
        case STATES::LOOKFORLINE:
            return F("");
        case STATES::LOOKFORCORNER:
            return F("");
        case STATES::BEANS:
            return F("");
        case STATES::BEANSGOBACK:
            return F("");
        case STATES::POOLSGOBACK:
            return F("");
        case STATES::LOOKFORLINEBACKWARDS:
            return F("");
        case STATES::BENEFITSSTARTCORNER:
            return F("");
        case STATES::BENEFITS:
            return F("");
        case STATES::STOP:
            return F("");
        default:
            return F("");
        }
    }

    const __FlashStringHelper *poolStateName(PoolSubState state)
    {
        switch (state)
        {
        case PoolSubState::FORWARD:
            return F("");
        case PoolSubState::AVOID_LEFT:
            return F("");
        case PoolSubState::AVOID_RIGHT:
            return F("");
        default:
            return F("");
        }
    }
}

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::START;
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = millis();
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    visionLeft = 0;
    visionRight = 0;

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

    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);

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
}

void LARCStateMachine::update()
{
    ir.update();
    qtrFront.update();
    vision.update();
    const uint32_t now = millis();
    startStateTime();

    const int linePos = qtrFront.getPosition();
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

    Serial.print(F(" ❤ Odometria❤ | X:"));    Serial.print(odomMove_.getX(),   3);
    Serial.print(F(" Y:"));      Serial.print(odomMove_.getY(),   3);
    Serial.print(F(" Yaw:"));    Serial.print(odomMove_.getThetaDeg(), 1);
    Serial.print(F(" UL:"));     Serial.print(odomMove_.getRpmUL(), 0);
    Serial.print(F(" UR:"));     Serial.print(odomMove_.getRpmUR(), 0);
    Serial.print(F(" LL:"));     Serial.print(odomMove_.getRpmLL(), 0);
    Serial.print(F(" LR:"));     Serial.print(odomMove_.getRpmLR(), 0);

    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState); Serial.print(")");
    Serial.print(F(" PS:")); Serial.print((int)poolState);

    Serial.print(F(" ❤ Tof❤ |"));
    Serial.print(F(" TL:")); Serial.print(tofLeft.getDistanceCm(), 0);
    Serial.print(F("cm vL:")); Serial.print(tofLeft.isValid() ? "OK" : "NO");
    Serial.print(F(" TR:")); Serial.print(tofRight.getDistanceCm(), 0);
    Serial.print(F("cm vR:")); Serial.print(tofRight.isValid() ? "OK" : "NO");

    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(FL);
    Serial.print(F(" FR:"));   Serial.print(FR);
    Serial.print(F(" BL:"));   Serial.print(BL);
    Serial.print(F(" BR:"));   Serial.print(BR);

    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(onLine);
    Serial.print(F(" lPos:"));  Serial.print(qtrFront.getPosition());
    Serial.print(F(" vx:")); Serial.print(vx);
    Serial.println();
    }

    const bool frontLeftDetectedLine = FL;
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BL;
    const bool backRightDetectedLine = BR;
    const bool frontDetectedLine = (FL || FR);
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

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
    obstacleDetectStartMs = 0;
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
        handleStartState(now, backDetected);
        break;

    case STATES::POOL:
        handlePoolState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case STATES::LOOKFORLINE:
        handleLookForLineState(now, frontDetectedLine, frontLeftDetectedLine, frontRightDetectedLine, onLine);
        break;

    case STATES::LOOKFORCORNER:
        handleLookForCornerState(now, backLeftDetectedLine, vx);
        break;

    case STATES::BEANS:
        handleBEANS(now, backRightDetectedLine, onLine, vx);
        break;

    case STATES::BEANSGOBACK:
        handleBEANSGoBackState(now, frontLeftDetectedLine, onLine, vx);
        break;

    case STATES::POOLSGOBACK:
        handlePOOLSGoBackState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case STATES::LOOKFORLINEBACKWARDS:
        handleLookForLineBackWards(now, backDetected, backLeftDetectedLine, backRightDetectedLine);
        break;

    case STATES::BENEFITSSTARTCORNER:
        handleBenefitsStartCorner(now, frontLeftDetectedLine, vx, onLine);
        break;

    case STATES::BENEFITS:
        handleBenefits(now, backLeftDetectedLine, vx, onLine);
        break;

    case STATES::STOP:
        handleStopState();
        break;

    default:
        handleStopState();
        break;
    }
}

void LARCStateMachine::setState(STATES newState)
{
    if (currentState == newState)
        return;

    currentState = newState;
    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    lfCorrecting        = false;
    lfCorrectionDir     = 0;
    lfCorrectionStartMs = 0;

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

void LARCStateMachine::setPoolState(PoolSubState newState)
{
    if (poolState == newState)
        return;

    poolState = newState;
    clearStartMs = 0;
    sideDetectStartMs = 0;
    noObstacleStartMs = 0;
    poolStateStartMs = millis();

    Serial.print(F("Pool substate -> "));
    Serial.println(poolStateName(poolState));
}

void LARCStateMachine::readVision()
{
    if (Serial.available() >= 3)
    {
        if (Serial.read() == 0xFF)
        {
            visionLeft = Serial.read();
            visionRight = Serial.read();
        }
    }
}

void LARCStateMachine::handleStartState(uint32_t now, bool backDetected)
{
    vision.stop();

    const bool limitPressed = (digitalRead(limitSwitch) == HIGH);

    if (limitPressed != lastLimitPressed)
    {
        if (limitPressed)
            Serial.println("LIMIT SWITCH PRESIONADO");
        else
            Serial.println("LIMIT SWITCH LIBERADO");

        lastLimitPressed = limitPressed;
    }

    switch (action_stage)
    {
    case 0:
        if (limitPressed)
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            action_start_time = now;
            action_stage = 1;
        }
        else
        {
            elevator.ElevatorPosition(2);
            odomMove_.stop();

            if ((now - action_start_time) >= 12000)
            {
                action_start_time = now;
                action_stage = 4;
            }
        }
        break;

    case 1:
        elevator.ElevatorPosition(1);
        odomMove_.stop();

        if (!limitPressed)
        {
            action_start_time = now;
            action_stage = 2;
        }
        break;

    case 2:
        elevator.ElevatorPosition(0);
        odomMove_.stop();

        if ((now - action_start_time) >= 2000)
        {
            action_start_time = now;
            action_stage = 0;
        }
        break;

    case 4:
        elevator.ElevatorPosition(0);
        odomMove_.stop();
        if ((now - action_start_time) >= 1500)
        {
            action_start_time = now;
            action_stage = 5;
        }
        break;

    case 5:
        elevator.ElevatorPosition(0);
        odomMove_.forward(50.0f);

        if ((now - action_start_time) >= kStartIgnoreTimeMs)
        {
            setPoolState(PoolSubState::FORWARD);
            setState(STATES::POOL);
        }
        break;
    }
}

void LARCStateMachine::handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected)
{
    vision.stop();
    vision.clearErrors();

    switch (poolState)
    {
    case PoolSubState::FORWARD:
    {
        static bool     lineCorrectionActive   = false;
        static uint32_t lineCorrectionStartMs  = 0;
        static int8_t   lineCorrectionDir      = 0;

        static constexpr uint32_t kLineCorrectionMs    = 120;
        static constexpr float    kLineCorrectionSpeed = 0.50f;
        static constexpr float    kNormalSpeed = 50.0f;

        if (lineCorrectionActive)
        {
            if ((now - lineCorrectionStartMs) < kLineCorrectionMs)
            {
                if (lineCorrectionDir < 0)
                    odomMove_.left(kNormalSpeed);
                else
                    odomMove_.right(kNormalSpeed);
                break;
            }
            lineCorrectionActive  = false;
            lineCorrectionStartMs = 0;
            lineCorrectionDir     = 0;
        }

        if (obstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
        }
        else
        {
            if (noObstacleStartMs == 0)
                noObstacleStartMs = now;

            if (rightDetected)
            {
                lineCorrectionActive  = true;
                lineCorrectionStartMs = now;
                lineCorrectionDir     = -1;
                odomMove_.left(kNormalSpeed);
                break;
            }
            else if (leftDetected)
            {
                lineCorrectionActive  = true;
                lineCorrectionStartMs = now;
                lineCorrectionDir     = +1;
                odomMove_.right(kNormalSpeed);
                break;
            }

            odomMove_.forward(kNormalSpeed);

            if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
            {
                setState(STATES::LOOKFORLINE);
            }
        }
        break;
    }

    case PoolSubState::AVOID_LEFT:
    {
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid()  && distL < 12.0f) ||
                              (tofRight.isValid() && distR < 12.0f);

        if (tooClose)
        {
            odomMove_.backward(53.0f);
            break;
        }

        odomMove_.left(55.0f);

        const bool justEntered = (now - poolStateStartMs) < 150;

        if (leftDetected && !justEntered)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_RIGHT);
        }
        else
        {
            sideDetectStartMs = 0;

            if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }
        }
        break;
    }

    case PoolSubState::AVOID_RIGHT:
    {
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid()  && distL < 10.0f) ||
                              (tofRight.isValid() && distR < 10.0f);

        if (tooClose)
        {
            odomMove_.backward(55.0f);
            break;
        }

        odomMove_.right(55.0f);

        const bool justEntered = (now - poolStateStartMs) < 150;

        if (rightDetected && !justEntered)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
        }
        else
        {
            sideDetectStartMs = 0;

            if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }
        }
        break;
    }
    }
}

void LARCStateMachine::handleLookForLineState(uint32_t now,
                                              bool frontDetected,
                                              bool leftDetected,
                                              bool rightDetected,
                                              bool onLine)
{
    if (action_stage == 0)
    {
        if (action_start_time == 0)
            action_start_time = now;

        odomMove_.backward(180.0f);

        if ((now - action_start_time) >= 200)
        {
            action_stage = 1;
            action_start_time = now;
        }
        return;
    }

    if (action_stage == 1)
    {
        odomMove_.backward(200.0f);

        if ((now - action_start_time) >= 200)
        {
            action_stage = 2;
            action_start_time = now;
        }
        return;
    }

    if (action_stage == 2)
    {
        odomMove_.forward(120.0f);

        if ((now - action_start_time) >= 300)
        {
            action_stage = 3;
            action_start_time = now;
        }
        return;
    }

    static constexpr uint32_t kBorderCorrectMs = 150;
    static constexpr float    kTofBorderCm     = 15.0f;

    const bool realBorderLeft  = leftDetected  && tofLeft.isValid()  && tofLeft.getDistanceCm()  > kTofBorderCm;
    const bool realBorderRight = rightDetected && tofRight.isValid() && tofRight.getDistanceCm() > kTofBorderCm;

    if (frontDetected && onLine)
    {
        lfCorrecting        = false;
        lfCorrectionDir     = 0;
        lfCorrectionStartMs = 0;
        lfLeftHoldMs        = 0;
        lfRightHoldMs       = 0;
        Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
        odomMove_.stop();
        setState(STATES::LOOKFORCORNER);
        return;
    }

    if (lfCorrecting)
    {
        if ((now - lfCorrectionStartMs) < kBorderCorrectMs)
        {
            if (lfCorrectionDir < 0)
                odomMove_.left(55.0f);
            else
                odomMove_.right(55.0f);
            return;
        }
        lfCorrecting = false;
        odomMove_.forward(50.0f);
        return;
    }

    if (realBorderLeft && !rightDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = +1;
        lfCorrectionStartMs = now;
        odomMove_.right(55.0f);
        return;
    }

    if (realBorderRight && !leftDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = -1;
        lfCorrectionStartMs = now;
        odomMove_.left(55.0f);
        return;
    }

    odomMove_.forward(50.0f);
}

void LARCStateMachine::handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx)
{
    static constexpr uint32_t kCornerStopMs = 8200;
    static constexpr uint32_t kSoftStartMs  = 500;

    switch (action_stage)
    {
    case 0:
    {
        if (cornerLEFTDetected)
        {
            odomMove_.stop();
            vision.startBeans();
            action_stage = 1;
            action_start_time = now;
            return;
        }
        const int error = 2900 - qtrFront.getPosition();
        const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
        odomMove_.setTranslation(-50.0f, corr);
        break;
    }

    case 1:
    {
        odomMove_.stop();

        if ((now - action_start_time) >= kCornerStopMs)
        {
            action_stage = 2;
            action_start_time = now;
        }
        break;
    }

    case 2:
    {
        odomMove_.stop();

        if ((now - action_start_time) >= kSoftStartMs)
        {
            setState(STATES::BEANS);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx)
{
    static constexpr uint32_t kLostLineTimeoutMs = 1200;

    if (vision.hasCriticalError())
    {
        vision.stop();
        setState(STATES::STOP);
        return;
    }

    switch (action_stage)
    {
    case 0:
    {
        if (cornerRIGHTDetected)
        {
            odomMove_.stop();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        if (!onLine)
        {
            if (action_start_time == 0)
                action_start_time = now;

            odomMove_.backward(58.0f);

            if ((now - action_start_time) >= kLostLineTimeoutMs)
            {
                vision.stop();
                setState(STATES::POOLSGOBACK);
            }

            return;
        }

        action_start_time = 0;

        const int error = 2200 - qtrFront.getPosition();
        const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
        odomMove_.setTranslation(+50.0f, corr);

        break;
    }

    case 1:
    {
        odomMove_.stop();
        if ((now - action_start_time) >= 1000)
        {
            action_start_time = 0;
            setState(STATES::BEANSGOBACK);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBEANSGoBackState(uint32_t now, bool frontLeftDetected, bool onLine, float vx)
{
    switch (action_stage)
    {
    case 0:
        vision.stop();
        vision.clearErrors();
        elevator.ElevatorPosition(0);
        odomMove_.stop();
        action_start_time = now;
        action_stage = 1;
        return;

    case 1:
        elevator.ElevatorPosition(0);
        odomMove_.stop();
        action_start_time = now;
        action_stage = 2;
        return;

    case 2:
        odomMove_.stop();

    case 3:
        elevator.ElevatorPosition(0);

        if (frontLeftDetected)
        {
            odomMove_.stop();
            action_start_time = now;
            action_stage = 4;
            return;
        }

        if (!onLine)
        {
            odomMove_.backward(55.0f);
        }
        else
        {
            const int error = 2900 - qtrFront.getPosition();
            const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
            odomMove_.setTranslation(-50.0f, corr);
        }
        return;

    case 4:
        elevator.ElevatorPosition(0);
        odomMove_.stop();

        if ((now - action_start_time) >= 8000)
        {
            vision.startBeans();
            setState(STATES::BEANS);
        }
        return;
    }
}

void LARCStateMachine::handlePOOLSGoBackState(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected)
{
switch (poolState)
{
case PoolSubState::FORWARD:
{
    if (rearObstacle)
    {
        noObstacleStartMs = 0;
        setPoolState(PoolSubState::AVOID_LEFT);
    }
    else
    {
        LARC.backward(kVelocity);

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINEBACKWARDS);
        }
    }
    break;
}

case PoolSubState::AVOID_LEFT:
{
    LARC.left(0.48f);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (leftDetected )
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_RIGHT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;
}

case PoolSubState::AVOID_RIGHT:
{
    LARC.right(0.48f);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (rightDetected)
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;
}
}
}

void LARCStateMachine::handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected)
{
    switch (action_stage)
    {
    case 0:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        if (backLeftDetected && !backRightDetected)
        {
            action_stage = 1;
            action_start_time = now;
            return;
        }

        if (backRightDetected && !backLeftDetected)
        {
            action_stage = 2;
            action_start_time = now;
            return;
        }

        odomMove_.backward(kBaseSpeed);
        break;
    }

    case 1:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        LARC.right(kVelocity);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
        }
        break;
    }

    case 2:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        LARC.left(kVelocity);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
        }
        break;
    }
    }
}

void LARCStateMachine::handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine)
{
    switch (action_stage)
    {
    case 0:
    {
        if (cornerLeftDetected)
        {
            LARC.brake();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        if (!onLine)
        {
            LARC.left(kBaseSpeed);
            return;
        }

        LARC.setTranslation(vx, 0.48f);

        break;
    }

    case 1:
    {
        LARC.brake();
        if ((now - action_start_time) >= 1000)
        {
            setState(STATES::BENEFITS);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBenefits(uint32_t now, bool cornerRIGHTDetected, float vx, bool online)
{
    switch (action_stage)
    {
    case 0:
    {
        if (!cornerRIGHTDetected)
        {
            action_stage = 1;
        }
        else
        {
            LARC.brake();
            action_start_time = now;
            action_stage = 2;
        }
        break;
    }

    case 1:
    {
        LARC.setTranslation(vx, -0.48f);

        if (cornerRIGHTDetected)
        {
            action_stage = 2;
        }
        break;
    }

    case 2:
    {
        LARC.brake();

        if ((now - action_start_time) >= 1000)
        {
            setState(STATES::STOP);
        }
        break;
    }
    }
}

void LARCStateMachine::handleStopState()
{
    LARC.brake();
}

void LARCStateMachine::updateControl()
{
    odomMove_.update();
}
