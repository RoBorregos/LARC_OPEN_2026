#include <Arduino.h>
#include "DriveStateMachineTest.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "Vision.hpp"

namespace
{
    static constexpr float kVelocity = 0.30f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    static constexpr float kCornerSetpoint = 2900.0f;
    static constexpr float kCornerKp = 0.00012f;
    static constexpr float kCornerCorrMax = 0.20f;

    static constexpr int kQtrPosMax = (QTR::N - 1) * 1000;
    static constexpr float kRearCorrAlpha = 0.15f;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs = 4500;
    static constexpr uint32_t kClearDelayMs = 1500;
    static constexpr uint32_t kNoObstacleToCornerMs = 1000;
    static constexpr uint32_t kCornerDeployWazitMs = 1800;

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTofTargetMm = 120;
    static constexpr uint16_t kTofHardStopMm = 105;
    static constexpr uint16_t kTofMaxRangeMm = 1000;
    static constexpr uint16_t kTooCloseAvoidLeftMm  = 120;
    static constexpr uint16_t kTooCloseAvoidRightMm = 100;

    static constexpr float kTofMinSpeed = 0.10f;
    static constexpr float kTofMaxSpeed = 0.35f;
    static constexpr uint16_t kObstacleDistanceMm = 215;

    static constexpr float kDistKp = 0.0012f;
    static constexpr float kDistKi = 0.0f;
    static constexpr float kDistKd = 0.00015f;

    static constexpr bool kLimitSwitchConnected = false;

    struct ObstacleLatch
    {
        static constexpr uint32_t kReleaseMs = 400;
        static constexpr uint32_t kConfirmMs = 0;

        bool     latched       = false;
        uint32_t clearStartMs  = 0;
        uint32_t detectStartMs = 0;

        bool update(uint32_t now, bool seenNow)
        {
            if (!latched)
            {
                if (!seenNow)
                {
                    detectStartMs = 0;
                    return false;
                }
                if (detectStartMs == 0)
                    detectStartMs = now;
                if ((now - detectStartMs) >= kConfirmMs)
                {
                    latched       = true;
                    clearStartMs  = 0;
                    detectStartMs = 0;
                }
                return latched;
            }

            if (seenNow)
            {
                clearStartMs = 0;
            }
            else
            {
                if (clearStartMs == 0)
                    clearStartMs = now;
                if ((now - clearStartMs) >= kReleaseMs)
                {
                    latched      = false;
                    clearStartMs = 0;
                }
            }
            return latched;
        }
    };

    const __FlashStringHelper *mainStateName(DriveTestSTATES state)
    {
        switch (state)
        {
        case DriveTestSTATES::START:
            return F("");
        case DriveTestSTATES::POOL:
            return F("");
        case DriveTestSTATES::LOOKFORLINE:
            return F("");
        case DriveTestSTATES::LOOKFORCORNER:
            return F("");
        case DriveTestSTATES::BEANS:
            return F("");
        case DriveTestSTATES::BEANSGOBACK:
            return F("");
        case DriveTestSTATES::POOLSGOBACK:
            return F("");
        case DriveTestSTATES::LOOKFORLINEBACKWARDS:
            return F("");
        case DriveTestSTATES::BENEFITSSTARTCORNER:
            return F("");
        case DriveTestSTATES::BENEFITS:
            return F("");
        case DriveTestSTATES::STOP:
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

DriveStateMachineTest::DriveStateMachineTest()
{
}

void DriveStateMachineTest::begin()
{
    currentState = DriveTestSTATES::POOL;
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

    Wire1.begin();
    Wire1.setClock(100000);
    Serial.print("i2cMux init: "); Serial.println(i2cMux.begin() ? "OK" : "FAIL");

    ToF* tofs[] = {&tofLeft, &tofRight, &tofBackLeft, &tofBackRight};
    const char* tofNames[] = {"tofLeft (UL)", "tofRight (UR)", "tofBackLeft (LL)", "tofBackRight (LR)"};
    for (uint8_t i = 0; i < 4; i++)
    {
        const bool ok = tofs[i]->begin();
        Serial.print(tofNames[i]); Serial.print(" init: "); Serial.println(ok ? "OK" : "FAIL");
        tofs[i]->setMaxRange(kTofMaxRangeMm);
        tofs[i]->setUpdateInterval(30);
    }

    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);

    ir.begin();
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    LARC.begin();
    LARC.holdYaw(true);
}

void DriveStateMachineTest::update()
{
    ir.update();
    tofLeft.update();
    tofRight.update();
    tofBackLeft.update();
    tofBackRight.update();
    qtrFront.update();
    qtrRear.update();
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

    lastVx_ = vx;
    debugPrint(true);

    const bool frontLeftDetectedLine = FL;
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BR;
    const bool backRightDetectedLine = BL;
    const bool frontDetectedLine = (FL || FR);
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    static constexpr uint32_t kTofWarmupMs = 500;
    static uint32_t tofReadyTimestamp = 0;
    if (tofReadyTimestamp == 0 &&
        (tofLeft.isValid() || tofRight.isValid() || tofBackLeft.isValid() || tofBackRight.isValid()))
        tofReadyTimestamp = now;
    const bool tofReady = tofReadyTimestamp != 0 &&
                        (now - tofReadyTimestamp) > kTofWarmupMs;
    tofReady_ = tofReady;

    auto seesObstacle = [&](const ToF& tof)
    {
        return tofReady && tof.isValid() && tof.getDistanceMm() < kObstacleDistanceMm;
    };

    static ObstacleLatch frontLatch;
    const bool obstacle = frontLatch.update(now, seesObstacle(tofLeft) || seesObstacle(tofRight));

    static ObstacleLatch rearLatch;
    const bool rearObstacle = rearLatch.update(now, seesObstacle(tofBackLeft) || seesObstacle(tofBackRight));

    switch (currentState)
    {
    case DriveTestSTATES::START:
        handleStartState(now, backDetected);
        break;

    case DriveTestSTATES::POOL:
        handlePoolState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case DriveTestSTATES::LOOKFORLINE:
        handleLookForLineState(now, frontDetectedLine, frontLeftDetectedLine, frontRightDetectedLine, onLine);
        break;

    case DriveTestSTATES::LOOKFORCORNER:
        handleLookForCornerState(now, backLeftDetectedLine, vx, onLine);
        break;

    case DriveTestSTATES::BEANS:
        handleBEANS(now, backRightDetectedLine, onLine, vx);
        break;

    case DriveTestSTATES::BEANSGOBACK:
        handleBEANSGoBackState(now, BL, BR, FL);
        break;

    case DriveTestSTATES::POOLSGOBACK:
        handlePOOLSGoBackState(now, rearObstacle, leftDetectedPool, rightDetectedPool);
        break;

    case DriveTestSTATES::LOOKFORLINEBACKWARDS:
        handleLookForLineBackWards(now, backDetected, backLeftDetectedLine, backRightDetectedLine);
        break;

    case DriveTestSTATES::BENEFITSSTARTCORNER:
        handleBenefitsStartCorner(now, frontLeftDetectedLine, vx, onLine);
        break;

    case DriveTestSTATES::BENEFITS:
        handleBenefits(now, frontRightDetectedLine, vx, onLine);
        break;

    case DriveTestSTATES::STOP:
        handleStopState();
        break;

    default:
        handleStopState();
        break;
    }
}

void DriveStateMachineTest::setState(DriveTestSTATES newState)
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

    cornerCorrFiltered = 0.0f;
    rearCorrFiltered = 0.0f;
    backLineArmed = false;
    backLineArmedMs = 0;

    qtrFront.resetFilter();
    qtrRear.resetFilter();

    vision.resetGuards();

    Serial.println(mainStateName(currentState));
}

void DriveStateMachineTest::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void DriveStateMachineTest::setPoolState(PoolSubState newState)
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

void DriveStateMachineTest::readVision()
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

void DriveStateMachineTest::debugPrint(bool enabled)
{
    if (!enabled)
        return;

    const uint32_t now = millis();
    static uint32_t debugPrintMs = 0;
    if ((now - debugPrintMs) < 100)
        return;
    debugPrintMs = now;

    Serial.print(F("DriveStateMachineTest"));

    Serial.print(F(" ❤ Yaw❤ | Deg:")); Serial.print(LARC.getYaw() * 180.0f / PI, 1);

    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState);
    Serial.print(F(" PS:")); Serial.print((int)poolState);
    Serial.print(F(" AS:")); Serial.print(action_stage);
    Serial.print(F(" LSW:")); Serial.print(digitalRead(limitSwitch));

    auto printTof = [](const __FlashStringHelper* label, const ToF& tof)
    {
        Serial.print(label);
        Serial.print(tof.isValid() ? (int)tof.getDistanceMm() : -1);
        Serial.print(F("mm"));
    };
    Serial.print(F(" ❤ ToF❤ |"));
    printTof(F(" UR:"), tofRight);
    printTof(F(" UL:"), tofLeft);
    printTof(F(" LL:"), tofBackLeft);
    printTof(F(" LR:"), tofBackRight);

    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(ir.getState(IRLine::FL));
    Serial.print(F(" FR:")); Serial.print(ir.getState(IRLine::FR));
    Serial.print(F(" BL:")); Serial.print(ir.getState(IRLine::BL));
    Serial.print(F(" BR:")); Serial.print(ir.getState(IRLine::BR));

    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(qtrFront.onLine());
    Serial.print(F(" lPos:")); Serial.print(qtrFront.getPosition());
    Serial.print(F(" vx:")); Serial.print(lastVx_);

    Serial.print(F(" ❤ qtrRear| onLine:")); Serial.print(qtrRear.onLine(Constants::QTRCalibration::kBinaryThreshold));
    Serial.print(F(" lPos:")); Serial.print(qtrRear.getPosition());

    auto printRawNorm = [](const __FlashStringHelper* rawLabel,
                           const __FlashStringHelper* normLabel,
                           const QTR& qtr)
    {
        const uint16_t* raw  = qtr.getRaw();
        const uint16_t* norm = qtr.getNorm();
        Serial.print(rawLabel);
        for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(raw[i]); Serial.print(','); }
        Serial.print(normLabel);
        for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(norm[i]); Serial.print(','); }
    };
    printRawNorm(F(" | raw:"), F(" norm:"), qtrFront);
    printRawNorm(F(" | rearRaw:"), F(" rearNorm:"), qtrRear);

    Serial.println();
}

void DriveStateMachineTest::handleStartState(uint32_t now, bool backDetected)
{
    vision.stop();

    const bool limitPressed = kLimitSwitchConnected && (digitalRead(limitSwitch) == HIGH);

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
            LARC.stop();
            action_start_time = now;
            action_stage = 1;
        }
        else
        {
            elevator.ElevatorPosition(2);
            LARC.stop();

            if ((now - action_start_time) >= 12000)
            {
                action_start_time = now;
                action_stage = 4;
            }
        }
        break;

    case 1:
        elevator.ElevatorPosition(1);
        LARC.stop();

        if (!limitPressed)
        {
            action_start_time = now;
            action_stage = 2;
        }
        break;

    case 2:
        elevator.ElevatorPosition(0);
        LARC.stop();

        if ((now - action_start_time) >= 2000)
        {
            action_start_time = now;
            action_stage = 0;
        }
        break;

    case 4:
        elevator.ElevatorPosition(0);
        LARC.stop();
        if ((now - action_start_time) >= 1500)
        {
            action_start_time = now;
            action_stage = 5;
        }
        break;

    case 5:
        elevator.ElevatorPosition(0);
        LARC.forward(0.30f);

        if ((now - action_start_time) >= kStartIgnoreTimeMs)
        {
            setPoolState(PoolSubState::FORWARD);
            setState(DriveTestSTATES::POOL);
        }
        break;
    }
}

void DriveStateMachineTest::handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected)
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
        static constexpr float    kLineCorrectionSpeed = 0.30f;
        static constexpr float    kNormalSpeed = 0.30f;

        if (lineCorrectionActive)
        {
            if ((now - lineCorrectionStartMs) < kLineCorrectionMs)
            {
                if (lineCorrectionDir < 0)
                    LARC.left(kNormalSpeed);
                else
                    LARC.right(kNormalSpeed);
                break;
            }
            lineCorrectionActive  = false;
            lineCorrectionStartMs = 0;
            lineCorrectionDir     = 0;
        }

        if (!tofReady_)
        {
            noObstacleStartMs = 0;
            LARC.stop();
            break;
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
                LARC.left(kNormalSpeed);
                break;
            }
            else if (leftDetected)
            {
                lineCorrectionActive  = true;
                lineCorrectionStartMs = now;
                lineCorrectionDir     = +1;
                LARC.right(kNormalSpeed);
                break;
            }

            LARC.forward(kNormalSpeed);

            if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
            {
                setState(DriveTestSTATES::LOOKFORLINE);
            }
        }
        break;
    }

    case PoolSubState::AVOID_LEFT:
    {
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool leftAlready = false;
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidLeftMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidLeftMm);

        if (tooClose)
        {
            LARC.backward(0.30f);
            break;
        }

        LARC.left(0.30f);

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
        const uint16_t distL = tofLeft.getDistanceMm();
        const uint16_t distR = tofRight.getDistanceMm();
        const bool tooClose = (tofLeft.isValid()  && distL < kTooCloseAvoidRightMm) ||
                              (tofRight.isValid() && distR < kTooCloseAvoidRightMm);

        if (tooClose)
        {
            LARC.backward(0.30f);
            break;
        }

        LARC.right(0.30f);

        const bool justEntered = (now - poolStateStartMs) < 100;

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

void DriveStateMachineTest::handleLookForLineState(uint32_t now,
                                              bool frontDetected,
                                              bool leftDetected,
                                              bool rightDetected,
                                              bool onLine)
{
    if (action_stage == 0)
    {
        if (action_start_time == 0)
            action_start_time = now;

        LARC.backward(0.30f);

        if ((now - action_start_time) >= 200)
        {
            action_stage = 1;
            action_start_time = now;
        }
        return;
    }

    if (action_stage == 1)
    {
        LARC.backward(0.30f);

        if ((now - action_start_time) >= 200)
        {
            action_stage = 2;
            action_start_time = now;
        }
        return;
    }

    if (action_stage == 2)
    {
        LARC.forward(0.30f);

        if ((now - action_start_time) >= 300)
        {
            action_stage = 3;
            action_start_time = now;
        }
        return;
    }

    static constexpr uint32_t kBorderCorrectMs = 150;
    static constexpr uint16_t kTofBorderMm      = 150;

    const bool realBorderLeft  = leftDetected  && tofLeft.isValid()  && tofLeft.getDistanceMm()  > kTofBorderMm;
    const bool realBorderRight = rightDetected && tofRight.isValid() && tofRight.getDistanceMm() > kTofBorderMm;

    if (frontDetected && onLine)
    {
        lfCorrecting        = false;
        lfCorrectionDir     = 0;
        lfCorrectionStartMs = 0;
        lfLeftHoldMs        = 0;
        lfRightHoldMs       = 0;
        Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
        LARC.stop();
        setState(DriveTestSTATES::LOOKFORCORNER);
        return;
    }

    if (lfCorrecting)
    {
        if ((now - lfCorrectionStartMs) < kBorderCorrectMs)
        {
            if (lfCorrectionDir < 0)
                LARC.left(0.30f);
            else
                LARC.right(0.30f);
            return;
        }
        lfCorrecting = false;
        LARC.forward(0.30f);
        return;
    }

    if (realBorderLeft && !rightDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = +1;
        lfCorrectionStartMs = now;
        LARC.right(0.30f);
        return;
    }

    if (realBorderRight && !leftDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = -1;
        lfCorrectionStartMs = now;
        LARC.left(0.30f);
        return;
    }

    LARC.forward(0.30f);
}

void DriveStateMachineTest::handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx, bool onLine)
{
    static constexpr uint32_t kCornerStopMs = 8200;
    static constexpr uint32_t kSoftStartMs  = 500;

    switch (action_stage)
    {
    case 0:
    {
        if (cornerLEFTDetected)
        {
            LARC.stop();
            vision.startBeans();
            action_stage = 1;
            action_start_time = now;
            return;
        }

        float corrTarget = 0.0f;
        if (onLine)
        {
            const float error = kCornerSetpoint - qtrFront.getPosition();
            corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
        }

        static constexpr float kCornerCorrAlpha = 0.15f;
        cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

        LARC.setTranslation(-cornerCorrFiltered, kVelocity);
        break;
    }

    case 1:
    {
        LARC.stop();

        if ((now - action_start_time) >= kCornerStopMs)
        {
            action_stage = 2;
            action_start_time = now;
        }
        break;
    }

    case 2:
    {
        LARC.stop();

        if ((now - action_start_time) >= kSoftStartMs)
        {
            setState(DriveTestSTATES::BEANS);
        }
        break;
    }
    }
}

void DriveStateMachineTest::handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx)
{
    static constexpr uint32_t kLostLineTimeoutMs = 1200;

    if (vision.hasCriticalError())
    {
        vision.stop();
        setState(DriveTestSTATES::STOP);
        return;
    }

    switch (action_stage)
    {
    case 0:
    {
        if (cornerRIGHTDetected)
        {
            LARC.stop();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        action_start_time = 0;

        float corrTarget = 0.0f;
        if (onLine)
        {
            const float error = kCornerSetpoint - qtrFront.getPosition();
            corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
        }

        static constexpr float kCornerCorrAlpha = 0.15f;
        cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

        LARC.setTranslation(-cornerCorrFiltered, -kVelocity);

        break;
    }

    case 1:
    {
        LARC.stop();
        if ((now - action_start_time) >= 1000)
        {
            action_start_time = 0;
            setState(DriveTestSTATES::POOLSGOBACK);
        }
        break;
    }
    }
}

void DriveStateMachineTest::handleBEANSGoBackState(uint32_t now, bool BL, bool BR, bool FL)
{
    switch (action_stage)
    {
    case 0:
        vision.stop();
        vision.clearErrors();
        elevator.ElevatorPosition(0);

        if (BR || BL)
        {
            LARC.stop();
            action_stage = 1;
            return;
        }

        LARC.backward(0.30f);
        return;

    case 1:
        elevator.ElevatorPosition(0);

        if (FL)
        {
            LARC.stop();
            action_stage = 2;
            return;
        }

        LARC.left(0.30f);
        return;

    case 2:
        elevator.ElevatorPosition(0);
        LARC.stop();
        return;
    }
}

void DriveStateMachineTest::handlePOOLSGoBackState(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected)
{
    static constexpr uint32_t kInitBackMs = 500;
    static constexpr uint32_t kInitLeftMs = 500;

    if (action_stage == 0)
    {
        if (action_start_time == 0)
            action_start_time = now;

        LARC.backward(kVelocity);

        if ((now - action_start_time) >= kInitBackMs)
        {
            action_stage = 1;
            action_start_time = now;
        }
        return;
    }

    if (action_stage == 1)
    {
        LARC.left(kVelocity);

        if ((now - action_start_time) >= kInitLeftMs)
        {
            action_stage = 2;
            action_start_time = 0;
        }
        return;
    }

switch (poolState)
{
case PoolSubState::FORWARD:
{
    if (!tofReady_)
    {
        noObstacleStartMs = 0;
        LARC.stop();
        break;
    }

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
            setState(DriveTestSTATES::LOOKFORLINEBACKWARDS);
        }
    }
    break;
}

case PoolSubState::AVOID_LEFT:
{
    const bool tooCloseRear = (tofBackLeft.isValid()  && tofBackLeft.getDistanceMm()  < kTooCloseAvoidLeftMm) ||
                              (tofBackRight.isValid() && tofBackRight.getDistanceMm() < kTooCloseAvoidLeftMm);
    if (tooCloseRear)
    {
        LARC.forward(0.30f);
        break;
    }

    LARC.left(0.30f);

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
    const bool tooCloseRear = (tofBackLeft.isValid()  && tofBackLeft.getDistanceMm()  < kTooCloseAvoidRightMm) ||
                              (tofBackRight.isValid() && tofBackRight.getDistanceMm() < kTooCloseAvoidRightMm);
    if (tooCloseRear)
    {
        LARC.forward(0.30f);
        break;
    }

    LARC.right(0.30f);

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

void DriveStateMachineTest::handleLookForLineBackWards(uint32_t now,
                                                    bool backDetected,
                                                    bool backLeftDetected,
                                                    bool backRightDetected)
{
    static constexpr uint32_t kBorderCorrectMs = 150;
    static constexpr uint16_t kTofBorderMm      = 150;

    const bool realBorderLeft  = backLeftDetected  && tofLeft.isValid()  && tofLeft.getDistanceMm()  > kTofBorderMm;
    const bool realBorderRight = backRightDetected && tofRight.isValid() && tofRight.getDistanceMm() > kTofBorderMm;

    static constexpr uint32_t kBackLineArmDelayMs = 700;

    if (backDetected && !backLineArmed)
    {
        backLineArmed   = true;
        backLineArmedMs = now;
    }

    const bool armDelayElapsed = backLineArmed && (now - backLineArmedMs) >= kBackLineArmDelayMs;

    if (armDelayElapsed && qtrRear.onLine(Constants::QTRCalibration::kBinaryThreshold))
    {
        lfCorrecting        = false;
        lfCorrectionDir     = 0;
        lfCorrectionStartMs = 0;
        lfLeftHoldMs        = 0;
        lfRightHoldMs       = 0;
        Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
        LARC.stop();
        setState(DriveTestSTATES::BENEFITSSTARTCORNER);
        return;
    }

    if (lfCorrecting)
    {
        if ((now - lfCorrectionStartMs) < kBorderCorrectMs)
        {
            if (lfCorrectionDir < 0)
                LARC.left(0.30f);
            else
                LARC.right(0.30f);
            return;
        }
        lfCorrecting = false;
        LARC.forward(0.30f);
        return;
    }

    if (realBorderLeft && !backRightDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = +1;
        lfCorrectionStartMs = now;
        LARC.right(0.30f);
        return;
    }

    if (realBorderRight && !backLeftDetected)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = -1;
        lfCorrectionStartMs = now;
        LARC.left(0.30f);
        return;
    }

    LARC.backward(0.30f);
}

void DriveStateMachineTest::handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine)
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

        float corrTarget = 0.0f;
        if (qtrRear.onLine())
        {
            const float mirroredRearPos = kQtrPosMax - qtrRear.getPosition();
            const float error = kCornerSetpoint - mirroredRearPos;
            corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
        }

        rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

        LARC.setTranslation(rearCorrFiltered, kVelocity);

        break;
    }

    case 1:
    {
        LARC.stop();
        if ((now - action_start_time) >= 1000)
        {
            setState(DriveTestSTATES::BENEFITS);
        }
        break;
    }
    }
}

void DriveStateMachineTest::handleBenefits(uint32_t now, bool cornerRIGHTDetected, float vx, bool online)
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
            LARC.stop();
            action_start_time = now;
            action_stage = 2;
        }
        break;
    }

    case 1:
    {
        float corrTarget = 0.0f;
        if (qtrRear.onLine())
        {
            const float mirroredRearPos = kQtrPosMax - qtrRear.getPosition();
            const float error = kCornerSetpoint - mirroredRearPos;
            corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
        }

        rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

        LARC.setTranslation(rearCorrFiltered, -kVelocity);

        if (cornerRIGHTDetected)
        {
            action_stage = 2;
        }
        break;
    }

    case 2:
    {
        LARC.stop();

        if ((now - action_start_time) >= 1000)
        {
            setState(DriveTestSTATES::STOP);
        }
        break;
    }
    }
}

void DriveStateMachineTest::handleStopState()
{
    LARC.stop();
}

void DriveStateMachineTest::updateControl()
{
    LARC.update();
}
