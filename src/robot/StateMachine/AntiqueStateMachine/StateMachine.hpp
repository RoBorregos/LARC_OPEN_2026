#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "constants.h"
#include "pins.h"
#include "testOdometry.hpp"
#include "robot/instances/instances.hpp"

#include "../States/TierOne/StartState.hpp"
#include "../States/TierOne/PoolState.hpp"
#include "../States/TierOne/LookForLineState.hpp"

enum class STATES
{
    START,
    POOL,
    LOOKFORLINE,
    LOOKFORCORNER,
    BEANS,
    BEANSGOBACK,
    POOLSGOBACK,
    LOOKFORLINEBACKWARDS,
    BENEFITSSTARTCORNER,
    BENEFITS,
    STOP
};

class LARCStateMachine
{
public:
    LARCStateMachine();

    void begin();
    void update();
    void updateControl();

private:
    STATES currentState = STATES::START;
    PoolSubState poolState = PoolSubState::FORWARD;

    uint32_t state_start_time = 0;
    uint32_t action_start_time = 0;
    int action_stage = 0;

    uint32_t clearStartMs = 0;
    uint32_t noObstacleStartMs = 0;

    byte visionLeft = 0;
    byte visionRight = 0;

    const int limitSwitch = Pins::kLimitSwitch;
    bool lastLimitPressed = false;
    bool limitWasPressed = false;
    bool elevatorGoingUpByLimit = false;
    uint32_t elevatorUpStartMs = 0;

    uint32_t poolStateStartMs = 0;
    uint32_t sideDetectStartMs = 0;

    bool     lfCorrecting        = false;
    int8_t   lfCorrectionDir     = 0;
    uint32_t lfCorrectionStartMs = 0;
    uint32_t lfLeftHoldMs        = 0;
    uint32_t lfRightHoldMs       = 0;

    void setState(STATES newState);
    void setPoolState(PoolSubState newState);
    void startStateTime();
    void readVision();

    void handleStartState(uint32_t now, bool backDetected);
    void handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineState(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected, bool onLine);
    void handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx);
    void handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx);
    void handleBEANSGoBackState(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx);
    void handlePOOLSGoBackState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected);
    void handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine);
    void handleBenefits(uint32_t now, bool cornerRightDetected, float vx, bool onLine);

    void handleStopState();
};

#endif
