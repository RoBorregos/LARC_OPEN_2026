#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

namespace Pins
{
    // CHASSIS MOTORS
    constexpr uint8_t kPwmPin[5] = {
        8,  // PWM_M1
        10, // PWM_M2 (pin del PWM de m3_ll en motor_test.cpp)
        9,  // PWM_M3 (pin del PWM de m2_ur en motor_test.cpp)
        11, // PWM_M4
        12  // PWM_M5 (elevator)
    };

    constexpr uint8_t kUpperMotors[4] = {
        35, // IN1_M3 (pin de m2_ur en motor_test.cpp)
        36, // IN2_M3
        40, // IN1_M4
        39, // IN2_M4
    };

    constexpr uint8_t kLowerMotors[4] = {
        37,  // IN1_M2 (pin de m3_ll en motor_test.cpp)
        38,  // IN2_M2
        33,  // IN1_M1
        34   // IN2_M1
    };

    constexpr uint8_t kElevator[2] = {
        4, // IN1_M5
        3  // IN2_M5
    };

    // ENCODERS
    constexpr uint8_t kEncoders[8] = {
        0,  // ENB_M1
        1,  // ENA_M1
        2,  // ENA_M2
        13, // ENB_M2
        32, // ENA_M3
        31, // ENB_M3
        21, // ENA_M4
        20  // ENB_M4
    };

    // LIMIT SWITCHES
    constexpr uint8_t kLimitSwitch  = 7; // Limit1
    constexpr uint8_t kLimitSwitch2 = 6; // Limit2 // placeholder: no usado aun en el codigo, confirmar su rol (elevator top/bottom, sorter, etc.)

    // 74HC4067 MULTIPLEXERS
    static constexpr uint8_t kMuxSig  = 26; // SIG_A0 -- mux1 (QTR front)
    static constexpr uint8_t kMuxSig2 = 22; // SIG_A1 -- mux2 (QTR rear)

    static constexpr uint8_t kMuxS0 = 27; // s0_MUX
    static constexpr uint8_t kMuxS1 = 28; // s1_MUX
    static constexpr uint8_t kMuxS2 = 29; // s2_MUX
    static constexpr uint8_t kMuxS3 = 30; // s3_MUX

    // QTR ARRAYS (mux channels)
    static constexpr uint8_t kQtrFrontFirstCh = 0; // C0..C6 (C7 no se usa, QTR::N=7)
    static constexpr uint8_t kQtrRearFirstCh  = 8; // C8..C13 (C14/C15 no se usan)
    static constexpr uint8_t kQtrRearCount    = 6; // C8 = first sensor to touch the line when going backwards

    // IR SENSORS
    static constexpr uint8_t kIrChFL = 15; // L1
    static constexpr uint8_t kIrChFR = 14; // L2
    static constexpr uint8_t kIrChBL = 15;//23; // ya no usado para L3 (ver kIrChBLMux)
    static constexpr uint8_t kIrChBR = 14;//41; // ya no usado para L4 (ver kIrChBRMux)

    static constexpr uint8_t kIrChBLMux = 7;  // back-left  -- free channel of the front block (C0..C7)
    static constexpr uint8_t kIrChBRMux = 15; // back-right -- free channel of the rear block (C8..C15)

    // TOF SENSORS (TCA9548A channels)
    static constexpr uint8_t kToFchFR = 0; // UR -- Front Right
    static constexpr uint8_t kToFchFL = 2; // UL -- Front Left
    static constexpr uint8_t kToFchBL = 3; // LL -- Back Left
    static constexpr uint8_t kToFchBR = 4; // LR -- Back Right

    // SERVOS
    constexpr uint8_t kServoPin1 = 13; // SERVO1 -- mismo net que "INT" (BNO), CONFIRMAR
    constexpr uint8_t kServoPin2 = 3;  // SERVO2 -- OJO: pin 3 ahora es IN2_M5 (elevator), no usar
    constexpr uint8_t kServoPin3 = 4;  // SERVO3 -- OJO: pin 4 ahora es IN1_M5 (elevator), no usar
    constexpr uint8_t kServoPin4 = 5;  // SERVO4
    constexpr uint8_t kServoPin5 = 2;  // SERVO5 -- mismo net que "RST" (BNO), CONFIRMAR
    // kServoPin6: placeholder, pin no identificado en el esquematico

    constexpr uint8_t kUpperIntakeServo = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kLowerIntakeServo = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kSeparatorServo   = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kBenefitServo     = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kHolderServo      = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6

    // BNO RST/INT
    static constexpr uint8_t kBnoRstReserved = 2;  // posible conflicto con SERVO5 -- confirmar
    static constexpr uint8_t kBnoIntReserved = 13; // posible conflicto con SERVO1 -- confirmar

    // LEGACY ULTRASONICS
    constexpr uint8_t kDistanceSensors[4][2] = {
        {35, 33},     // FRONT LEFT  {TRIG, ECHO}
        {36, 34},     // FRONT RIGHT {TRIG, ECHO}
        {255, 255},   // BACK RIGHT not confirmed
        {255, 255}    // BACK LEFT  not confirmed
    };

    // LEGACY
    // constexpr uint8_t kBluetoothRx = 7;
    // constexpr uint8_t kBluetoothTx = 8;
}

#endif
