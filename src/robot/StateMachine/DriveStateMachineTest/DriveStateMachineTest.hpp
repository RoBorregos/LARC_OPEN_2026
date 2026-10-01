#ifndef DRIVESTATEMACHINETEST_H
#define DRIVESTATEMACHINETEST_H

#include <Arduino.h>
#include "constants.h"
#include "pins.h"
#include "robot/instances/instances.hpp"

#include "../States/TierOne/StartState.hpp"
#include "../States/TierOne/PoolState.hpp"
#include "../States/TierOne/LookForLineState.hpp"

enum class DriveTestSTATES
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

class DriveStateMachineTest
{
public:
    DriveStateMachineTest();

    void begin();
    void update();
    void updateControl();

private:
    DriveTestSTATES currentState = DriveTestSTATES::START;
    PoolSubState poolState = PoolSubState::FORWARD;

    uint32_t state_start_time = 0;
    uint32_t action_start_time = 0;
    int action_stage = 0;

    uint32_t clearStartMs = 0;
    uint32_t noObstacleStartMs = 0;

    bool tofReady_ = false;

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

    float cornerCorrFiltered = 0.0f;

    float rearCorrFiltered = 0.0f;

    bool     backLineArmed   = false;
    uint32_t backLineArmedMs = 0;

    void setState(DriveTestSTATES newState);
    void setPoolState(PoolSubState newState);
    void startStateTime();
    void readVision();

    void debugPrint(bool enabled);
    float lastVx_ = 0.0f;

    void handleStartState(uint32_t now, bool backDetected);
    void handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineState(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected, bool onLine);
    void handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx, bool onLine);
    void handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx);
    void handleBEANSGoBackState(uint32_t now, bool BL, bool BR, bool FL);
    void handlePOOLSGoBackState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected);
    void handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine);
    void handleBenefits(uint32_t now, bool cornerRightDetected, float vx, bool onLine);

    void handleStopState();
};

#endif
