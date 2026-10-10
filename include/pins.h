#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

namespace Pins
{
    // =========================================================
    // CHASSIS MOTORS
    // =========================================================
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

    // =========================================================
    // ELEVATOR
    // =========================================================

    // Moved off 16/17 (SCL1/SDA1, the TCA9548A / PCA9685 bus) in dev-RTOS.
    constexpr uint8_t kElevator[2] = {
        4, // IN1_M5
        3  // IN2_M5
    };

    // =========================================================
    // ENCODERS
    // kEncoders = {B1, A1, A2, B2, A3, B3, A4, B4}
    // =========================================================
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

    // =========================================================
    // LIMIT SWITCH
    // =========================================================
    constexpr uint8_t kLimitSwitch  = 7; // Limit1
    constexpr uint8_t kLimitSwitch2 = 6; // Limit2 // placeholder: unused for now, to confirm role

    // =========================================================
    // 74HC4067 MULTIPLEXERS
    // shared S0-S3, different SIG
    // =========================================================
    static constexpr uint8_t kMuxSig  = 26; // SIG_A0 -- mux1 (QTR front)
    static constexpr uint8_t kMuxSig2 = 22; // SIG_A1 -- mux2 (QTR rear)

    static constexpr uint8_t kMuxS0 = 27; // s0_MUX
    static constexpr uint8_t kMuxS1 = 28; // s1_MUX
    static constexpr uint8_t kMuxS2 = 29; // s2_MUX
    static constexpr uint8_t kMuxS3 = 30; // s3_MUX

    // =========================================================
    // QTR ARRAYS ON MUX1
    // =========================================================
    static constexpr uint8_t kQtrFrontFirstCh = 0; // C0..C6 (C7 no se usa, QTR::N=7)
    static constexpr uint8_t kQtrRearFirstCh  = 8; // C8..C14 (C15 unused, same pattern as front: QTR::N=7)

    // =========================================================
    // IR SENSORS ON MUX2
    // =========================================================
    static constexpr uint8_t kIrChFL = 15; // L1
    static constexpr uint8_t kIrChFR = 14; // L2
    static constexpr uint8_t kIrChBL = 15;//23; // unused for  L3 (see kIrChBLMux)
    static constexpr uint8_t kIrChBR = 14;//41; // not used for L4 anymore (see kIrChBRMux)

    static constexpr uint8_t kIrChBLMux = 15; // L3 -- free channel of rear block (C8..C15)
    static constexpr uint8_t kIrChBRMux = 7;  // L4 -- free channel of front block  (C0..C7)

    // =========================================================
    // I2C MULTIPLEXER (TCA9548A) — shared by the ToFs and the PCA9685
    // =========================================================
    namespace I2cMux
    {
        constexpr uint8_t kAddress = 0x70; // A0-A2 to GND
        constexpr uint8_t kBus     = 1;    // 0 = Wire, 1 = Wire1 (SDA 17 / SCL 16), 2 = Wire2
    } // namespace I2cMux

    // =========================================================
    // TOF SENSORS ON I2C MUX (TCA9548A)
    // =========================================================
    static constexpr uint8_t kToFchFR = 0; // UR -- Front Right
    static constexpr uint8_t kToFchFL = 2; // UL -- Front Left
    static constexpr uint8_t kToFchBL = 3; // LL -- Back Left
    static constexpr uint8_t kToFchBR = 4; // LR -- Back Right

    // =========================================================
    // SERVOS — PCA9685 16-channel PWM driver (generic board)
    // =========================================================
    namespace Servos
    {
        // The PCA9685 board  
        constexpr uint8_t kPcaI2cAddress = 0x40; // A0-A5 jumpers open
        constexpr uint8_t kI2cBus        = 1;   // 0 = Wire, 1 = Wire1, 2 = Wire2
        constexpr uint8_t kI2cSda        = 17;  // SDA1
        constexpr uint8_t kI2cScl        = 16;   // SCL1
        constexpr uint8_t kOePin         = 255;  // OE (active LOW); 255 = not wired

        // TCA9548A channel the PCA9685 is plugged into (0-7).
        // 255 = PCA9685 wired straight to the bus, no multiplexer.
        constexpr uint8_t kTcaChannel    = 1;

        static_assert(kTcaChannel <= 7 || kTcaChannel == 255,
                      "Pins::Servos::kTcaChannel must be 0-7, or 255 for no mux");
        static_assert(kTcaChannel == 255 || kI2cBus == I2cMux::kBus,
                      "Pins::Servos: PCA9685 is behind the TCA9548A, so kI2cBus must equal I2cMux::kBus");

        static_assert((kI2cBus == 0 && kI2cSda == 18 && kI2cScl == 19) ||
                      (kI2cBus == 1 && kI2cSda == 17 && kI2cScl == 16) ||
                      (kI2cBus == 2 && kI2cSda == 25 && kI2cScl == 24),
                      "Pins::Servos: kI2cBus does not match kI2cSda/kI2cScl");

        // (PROVISIONAL (missing full test of all servos)
        constexpr uint8_t kIntakeUpperCh = 12;
        constexpr uint8_t kIntakeLowerCh = 8;
        constexpr uint8_t kSeparatorCh   = 4;
        constexpr uint8_t kBenefit1Ch    = 15;
        constexpr uint8_t kBenefit2Ch    = 0;

        constexpr uint8_t kChannels[] = {
            kIntakeUpperCh, kIntakeLowerCh, kSeparatorCh, kBenefit1Ch, kBenefit2Ch
        };

        constexpr bool channelsValid(uint8_t i = 0, uint8_t j = 1)
        {
            return i >= sizeof(kChannels)       ? true
                 : kChannels[i] > 15            ? false
                 : j >= sizeof(kChannels)       ? channelsValid(i + 1, i + 2)
                 : kChannels[i] == kChannels[j] ? false
                                                : channelsValid(i, j + 1);
        }
        static_assert(channelsValid(),
                      "Pins::Servos: a channel is > 15 or two servos share one");
    } // namespace Servos

    // =========================================================
    // BNO055 -- RST/INT
    // =========================================================
    static constexpr uint8_t kBnoRstReserved = 2;  // possible conflict with SERVO5 -- confirm
    static constexpr uint8_t kBnoIntReserved = 13; // possible conflict withSERVO1 -- confirm



    // =========================================================
    // LEGACY ULTRASONICS
    // =========================================================
    constexpr uint8_t kDistanceSensors[4][2] = {
        {35, 33},     // FRONT LEFT  {TRIG, ECHO}
        {36, 34},     // FRONT RIGHT {TRIG, ECHO}
        {255, 255},   // BACK RIGHT not confirmed
        {255, 255}    // BACK LEFT  not confirmed
    };

    // =========================================================
    // OPTIONAL / LEGACY
    // =========================================================
    // constexpr uint8_t kBluetoothRx = 7;
    // constexpr uint8_t kBluetoothTx = 8;
}

#endif