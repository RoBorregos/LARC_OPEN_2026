#include <Arduino.h>
#include "StateMachine.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "Vision.hpp"

namespace
{
    static constexpr STATES kInitialState = STATES::LOOKFORLINE;

    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START: // ST:0
            return F(""); // F("START ♡ ♡ ♡");
        case STATES::POOL: // ST:1
            return F(""); // F("POOL");
        case STATES::LOOKFORLINE: // ST:2
            return F(""); // F("LOOKFORLINE");
        case STATES::LOOKFORCORNER: // ST:3
            return F(""); // F("LOOKFORCORNER");
        case STATES::BEANS: // ST:4
            return F(""); // F("BEANS");
        case STATES::BEANSGOBACK: // ST:5
            return F(""); // F("BEANSGOBACK");
        case STATES::POOLSGOBACK: // ST:6
            return F(""); // F("POOLSGOBACK");
        case STATES::LOOKFORLINEBACKWARDS: // ST:7
            return F(""); // F("LOOKFORLINEBACKWARDS");
        case STATES::BENEFITSSTARTCORNER: // ST:8
            return F(""); // F("BENEFITSSTARTCORNER");
        case STATES::BENEFITS: // ST:9
            return F(""); // F("BENEFITS");
        case STATES::STOP: // ST:10
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
    currentState = kInitialState;
    state_start_time = millis();

    visionLeft = 0;
    visionRight = 0;

    //Elevator
    pinMode(limitSwitch, INPUT);
    elevator.begin();

    vision.begin();
    vision.requestStatus();

    Wire.begin();
    Wire.setClock(400000);

    Wire1.begin();
    Wire1.setClock(100000);
    Serial.print("i2cMux init: "); Serial.println(i2cMux.begin() ? "OK" : "FAIL");

    // UL/UR (frente) para POOL, LL/LR (atras) para POOLSGOBACK
    ToF* tofs[] = {&tofLeft, &tofRight, &tofBackLeft, &tofBackRight};
    const char* tofNames[] = {"tofLeft (UL)", "tofRight (UR)", "tofBackLeft (LL)", "tofBackRight (LR)"};
    for (uint8_t i = 0; i < 4; i++)
    {
        const bool ok = tofs[i]->begin();
        Serial.print(tofNames[i]); Serial.print(" init: "); Serial.println(ok ? "OK" : "FAIL");
        tofs[i]->setMaxRange(Perception::kTofMaxRangeMm);
        tofs[i]->setUpdateInterval(30);
    }

    //QTR
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);   // FRONT qtr

    ir.begin();
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    LARC.begin();
    LARC.holdYaw(true);

    beginState(currentState);
}

void LARCStateMachine::update()
{
    const uint32_t now = millis();
    perception_.update(now);
    vision.update();
    startStateTime();

    const PerceptionSnapshot &p = perception_.get();

    debugPrint(true); // true = imprime, false = no imprime

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
        poolState_.update(now, p.obstacle, p.leftDetectedPool, p.rightDetectedPool, p.tofReady, transitionToLookForLine);
        if (transitionToLookForLine)
            setState(STATES::LOOKFORLINE);
        break;
    }

    case STATES::LOOKFORLINE:
    {
        bool transitionToCorner = false;
        lookForLineState_.update(now, p.FL, p.FR, p.BL, p.BR, transitionToCorner);
        if (transitionToCorner)
            setState(STATES::LOOKFORCORNER);
        break;
    }

    case STATES::LOOKFORCORNER:
    {
        bool transitionToBeans = false;
        lookForCornerState_.update(now, p.backLeftDetectedLine, p.onLine, transitionToBeans);
        if (transitionToBeans)
            setState(STATES::BEANS);
        break;
    }

    case STATES::BEANS:
    {
        bool transitionToPoolsGoBack = false;
        bool transitionToStop = false;
        beansState_.update(now, p.backRightDetectedLine, p.onLine, transitionToPoolsGoBack, transitionToStop);
        if (transitionToStop)
            setState(STATES::STOP);
        else if (transitionToPoolsGoBack)
            setState(STATES::POOLSGOBACK);
        break;
    }

    case STATES::BEANSGOBACK: //(17-09-2026) We are skipping this step by the moment
        beansGoBackState_.update(p.BL, p.BR, p.FL);
        break;

    case STATES::POOLSGOBACK:
    {
        bool transitionToLookForLineBackwards = false;
        poolsGoBackState_.update(now, p.rearObstacle, p.leftDetectedPool, p.rightDetectedPool, p.tofReady, transitionToLookForLineBackwards);
        if (transitionToLookForLineBackwards)
            setState(STATES::LOOKFORLINEBACKWARDS);
        break;
    }

    case STATES::LOOKFORLINEBACKWARDS:
    {
        bool transitionToBenefitsStartCorner = false;
        lookForLineBackwardsState_.update(now, p.FL, p.FR, p.BL, p.BR, transitionToBenefitsStartCorner);
        if (transitionToBenefitsStartCorner)
            setState(STATES::BENEFITSSTARTCORNER);
        break;
    }

    case STATES::BENEFITSSTARTCORNER:
    {
        bool transitionToBenefits = false;
        benefitsStartCornerState_.update(now, p.frontLeftDetectedLine, transitionToBenefits);
        if (transitionToBenefits)
            setState(STATES::BENEFITS);
        break;
    }

    case STATES::BENEFITS:
    {
        bool transitionToStop = false;
        benefitsState_.update(now, p.frontRightDetectedLine, transitionToStop);
        if (transitionToStop)
            setState(STATES::STOP);
        break;
    }

    case STATES::STOP:
    default:
        stopState_.update();
        break;
    }
}

void LARCStateMachine::setState(STATES newState)
{
    if (currentState == newState)
        return;

    currentState = newState;
    state_start_time = millis();

    beginState(newState);

    qtrFront.resetFilter();
    qtrRear.resetFilter();

    vision.resetGuards();

    Serial.println(mainStateName(currentState));
}

void LARCStateMachine::beginState(STATES state)
{
    switch (state)
    {
    case STATES::START:                startState_.begin(); break;
    case STATES::POOL:                 poolState_.begin(); break;
    case STATES::LOOKFORLINE:          lookForLineState_.begin(); break;
    case STATES::LOOKFORCORNER:        lookForCornerState_.begin(); break;
    case STATES::BEANS:                beansState_.begin(); break;
    case STATES::BEANSGOBACK:          beansGoBackState_.begin(); break;
    case STATES::POOLSGOBACK:          poolsGoBackState_.begin(); break;
    case STATES::LOOKFORLINEBACKWARDS: lookForLineBackwardsState_.begin(); break;
    case STATES::BENEFITSSTARTCORNER:  benefitsStartCornerState_.begin(); break;
    case STATES::BENEFITS:             benefitsState_.begin(); break;
    case STATES::STOP:                 stopState_.begin(); break;
    }
}

void LARCStateMachine::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::debugPrint(bool enabled)
{
    if (!enabled)
        return;

    const uint32_t now = millis();
    static uint32_t debugPrintMs = 0;
    if ((now - debugPrintMs) < 100)
        return;
    debugPrintMs = now;

    const PerceptionSnapshot &p = perception_.get();

    Serial.print(F("LARCStateMachine"));

    Serial.print(F(" ❤ Yaw❤ | Deg:")); Serial.print(LARC.getYaw() * 180.0f / PI, 1);

    // Estado actual
    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState);
    Serial.print(F(" PS:"));
    Serial.print(currentState == STATES::POOLSGOBACK ? (int)poolsGoBackState_.getSubState()
                                                     : (int)poolState_.getSubState());
    Serial.print(F(" LSW:")); Serial.print(digitalRead(limitSwitch));

    // ToF en mm (-1 = sin lectura valida)
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

    // IR
    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(p.FL);
    Serial.print(F(" FR:")); Serial.print(p.FR);
    Serial.print(F(" BL:")); Serial.print(p.BL);
    Serial.print(F(" BR:")); Serial.print(p.BR);

    // Línea (lPos sirve para saber el valor del centro del qtr)
    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(p.onLine);
    Serial.print(F(" lPos:")); Serial.print(p.linePos);
    Serial.print(F(" vx:")); Serial.print(p.vx);

    Serial.print(F(" ❤ qtrRear| onLine:")); Serial.print(qtrRear.onLine(Constants::QTRCalibration::kBinaryThreshold));
    Serial.print(F(" lPos:")); Serial.print(qtrRear.getPosition());

    auto printRawNorm = [](const __FlashStringHelper* rawLabel,
                           const __FlashStringHelper* normLabel,
                           const QTR& qtr)
    {
        const uint16_t* raw  = qtr.getRaw();
        const uint16_t* norm = qtr.getNorm();
        Serial.print(rawLabel);
        for (uint8_t i = 0; i < qtr.size(); i++) { Serial.print(raw[i]); Serial.print(','); }
        Serial.print(normLabel);
        for (uint8_t i = 0; i < qtr.size(); i++) { Serial.print(norm[i]); Serial.print(','); }
    };
    printRawNorm(F(" | raw:"), F(" norm:"), qtrFront);
    printRawNorm(F(" | rearRaw:"), F(" rearNorm:"), qtrRear);

    Serial.println();
}

void LARCStateMachine::updateControl()
{
    LARC.update();
}
