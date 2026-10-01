#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "constants.h"
#include "pins.h"
#include "robot/instances/instances.hpp"
#include "robot/Perception/Perception.hpp"

// State machine (states) files...
#include "States/TierOne/StartState.hpp"
#include "States/TierOne/PoolState.hpp"
#include "States/TierOne/LookForLineState.hpp"
#include "States/TierTwo/LookForCornerState.hpp"
#include "States/TierTwo/BeansState.hpp"
#include "States/TierTwo/BeansGoBack.hpp"
#include "States/TierTwo/PoolsGoBackState.hpp"
#include "States/TierThree/LookForLineBackwards.hpp"
#include "States/TierThree/BenefitsStartCorner.hpp"
#include "States/TierThree/BenefitsState.hpp"
#include "States/TierThree/StopState.hpp"

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

class LARCStateMachine
{
public:
    LARCStateMachine();

    void begin();
    void update();
    void updateControl();

private:
    STATES currentState = STATES::START;

    uint32_t state_start_time = 0;

    byte visionLeft = 0;
    byte visionRight = 0;

    // ELEVATOR
    const int limitSwitch = Pins::kLimitSwitch;

    Perception perception_;

    StartState startState_;
    PoolState poolState_;
    LookForLineState lookForLineState_;
    LookForCornerState lookForCornerState_;
    BeansState beansState_;
    BeansGoBackState beansGoBackState_;
    PoolsGoBackState poolsGoBackState_;
    LookForLineBackwardsState lookForLineBackwardsState_;
    BenefitsStartCornerState benefitsStartCornerState_;
    BenefitsState benefitsState_;
    StopState stopState_;

    void setState(STATES newState);
    void beginState(STATES state);
    void startStateTime();

    // Todos los prints de debug: debugPrint(true) imprime, debugPrint(false) no.
    void debugPrint(bool enabled);
};

#endif
