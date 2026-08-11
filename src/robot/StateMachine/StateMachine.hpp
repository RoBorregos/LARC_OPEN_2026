#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "constants.h"
#include "pins.h"
#include "testOdometry.hpp"
#include "robot/instances/instances.hpp"

enum class STATES
{
    START,
    POOL,         
    LOOKFORLINE,          //Try if this works well, in case there aren't pools (add qtr for front line detection)
    LOOKFORCORNER,
    BEANS,                // BEANS(left to right) :: Recoje las pelotas y inicializa vision + Sorter (vision)
    BEANSGOBACK,          // BEANS(right to left) :: Elevator goes down :: Recoje las pelotas y inicializa vision (stop when corner detected) + Sorter (vision) +
    POOLSGOBACK,          // Avoid Pools but using the back US
    LOOKFORLINEBACKWARDS, // Look for the backwards line
    BENEFITSSTARTCORNER,  // Look for left corner
    BENEFITS,             // Rear Vision + liberating cacaos
    STOP                  // FINISH ALL TASKS      :D          !!! Ends in right corner
};

enum class PoolSubState
{
    FORWARD,
    AVOID_LEFT,
    AVOID_RIGHT
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

    // ELEVATOR
    const int limitSwitch = Pins::kLimitSwitch;
    bool lastLimitPressed = false;
    bool limitWasPressed = false;
    bool elevatorGoingUpByLimit = false;
    uint32_t elevatorUpStartMs = 0;

    // IRs to change of left or right
    uint32_t poolStateStartMs = 0;
    uint32_t sideDetectStartMs = 0;

    // lateral correction for LOOKFORLINE
    bool     lfCorrecting        = false;
    int8_t   lfCorrectionDir     = 0;
    uint32_t lfCorrectionStartMs = 0;
    uint32_t lfLeftHoldMs        = 0;  
    uint32_t lfRightHoldMs       = 0;  

    OdomMovement odomMove_;

    // Set states
    void setState(STATES newState);
    void setPoolState(PoolSubState newState);
    void startStateTime();
    void readVision();

    // Cases
    // First part
    void handleStartState(uint32_t now, bool backDetected);                                                                  // START
                                                                                                          // Second part (AVOID)
    void handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);             // POOLS
    void handleLookForLineState(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected, bool onLine); // LOOKFORLINE(no obstacle detected | obstacle no longer detected)
    void handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx);                       // LOOKFORCORNER (to start vision)
    // Third part (RECOLECT)
    void handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx); // BEANS state (recolection + sorting)
    void handleBEANSGoBackState(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx);
    // Fourth part (GO BACK)
    void handlePOOLSGoBackState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected);
    void handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine);
    void handleBenefits(uint32_t now, bool cornerRightDetected, float vx, bool onLine);

    // The End ♡
    void handleStopState(); // END of tasks :D !!!
};

#endif
