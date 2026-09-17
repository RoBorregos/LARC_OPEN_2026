#ifndef DRIVESTATEMACHINETEST_H
#define DRIVESTATEMACHINETEST_H

#include <Arduino.h>
#include "constants.h"
#include "pins.h"
#include "robot/instances/instances.hpp"

//State machine (states) files...
#include "../States/TierOne/StartState.hpp"
#include "../States/TierOne/PoolState.hpp"
#include "../States/TierOne/LookForLineState.hpp"

// Exactamente la misma máquina de estados que AntiqueStateMachine, pero
// moviendo con LARC (Drive: BNO085 yaw-hold + omni open-loop) en vez de
// odomMove_ (OdomMovement: PID de RPM cerrado por rueda). Sirve para probar
// los estados (IR, VLX/ToF, QTR) tal cual, sin depender de la odometría
// cerrada de OdomMovement.
enum class DriveTestSTATES
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

    // Filtro paso-bajo para el corr lateral de LOOKFORCORNER: lPos trae
    // ruido (EMI de motores) que hace saltar el error crudo entre +/-max
    // en menos de un ms; sin suavizar, ese ruido pasa directo al comando
    // de strafe y el robot vibra de lado en vez de corregir. Se resetea
    // en setState() al entrar de nuevo al estado.
    float cornerCorrFiltered = 0.0f;

    // Mismo filtro paso-bajo pero para la corrección qtrRear de
    // LOOKFORLINEBACKWARDS. Se resetea en setState() igual que la de arriba.
    float rearCorrFiltered = 0.0f;

    // LOOKFORLINEBACKWARDS: backDetected (L3/L4, IR traseros) y
    // qtrRear.onLine() estan desfasados -- los IR disparan un poco antes
    // que el QTR llegue a onLine(), asi que un AND simultaneo nunca se
    // cumple. Se arma este latch en cuanto backDetected dispara una vez y
    // la transicion a BENEFITSSTARTCORNER espera a qtrRear.onLine()
    // despues de armado, en vez de exigir ambos al mismo tiempo. Se
    // resetea en setState() al (re)entrar al estado.
    bool     backLineArmed   = false;
    uint32_t backLineArmedMs = 0;

    // Set states
    void setState(DriveTestSTATES newState);
    void setPoolState(PoolSubState newState);
    void startStateTime();
    void readVision();

    // Cases
    // First part
    void handleStartState(uint32_t now, bool backDetected);                                                                  // START
                                                                                                          // Second part (AVOID)
    void handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);             // POOLS
    void handleLookForLineState(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected, bool onLine); // LOOKFORLINE(no obstacle detected | obstacle no longer detected)
    void handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx, bool onLine);          // LOOKFORCORNER (to start vision)
    // Third part (RECOLECT)
    void handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx); // BEANS state (recolection + sorting)
    void handleBEANSGoBackState(uint32_t now, bool BL, bool BR, bool FL);
    // Fourth part (GO BACK)
    void handlePOOLSGoBackState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected);
    void handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected);
    void handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine);
    void handleBenefits(uint32_t now, bool cornerRightDetected, float vx, bool onLine);

    // The End ♡
    void handleStopState(); // END of tasks :D !!!
};

#endif
