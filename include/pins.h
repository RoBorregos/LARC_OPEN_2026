#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

// =============================================================
// pins.h ACTUALIZADO (2026-08-14) para la PCB nueva.
//
// La PCB nueva es parecida a la anterior pero cambio el mapeo de pines.
// Los valores de abajo salen del esquematico compartido por el equipo
// ese dia. Donde el esquematico no daba suficiente informacion para
// estar seguros (que pin fisico corresponde a que rol logico), se dejo
// un comentario "// placeholder" explicando que falta confirmar.
//
// IMPORTANTE: varias constantes de este archivo (kElevator*, los
// servos con nombre) hoy NO estan conectadas a sus clases consumidoras
// (Elevator, ServoSystem parecen hardcodear sus propios pines
// internamente) -- ver nota en cada seccion. Actualizarlas aqui no
// cambia el comportamiento real hasta que esas clases se revisen por
// separado.
// =============================================================

namespace Pins
{
    // =========================================================
    // CHASSIS MOTORS (M1-M4) -- confirmado por el equipo.
    // IN1_Mx / IN2_Mx / PWM_Mx, M1=upper-left, M2=upper-right,
    // M3=lower-left, M4=lower-right (mismo orden que la version anterior
    // de este archivo).
    // =========================================================
    constexpr uint8_t kPwmPin[5] = {
        8,  // PWM_M1
        9,  // PWM_M2
        10, // PWM_M3
        11, // PWM_M4
        12  //PWM_M5 (elevator)
    };

    constexpr uint8_t kUpperMotors[4] = {
        33, // IN1_M1 (UPPER LEFT, m1)
        34, // IN2_M1
        35, // IN1_M2 (UPPER RIGHT, m2)
        36  // IN2_M2
    };

    constexpr uint8_t kLowerMotors[4] = {
        37, // IN1_M3 (LOWER LEFT, m3)
        38, // IN2_M3
        39, // IN1_M4 (LOWER RIGHT, m4)
        40  // IN2_M4
    };

    constexpr uint8_t kElevator[2] = {
        16, //IN1_M5
        17  //IN2_M5
    };

    // =========================================================
    // ENCODERS (ENA_Mx / ENB_Mx en el esquematico = canales A/B por motor)
    // kEncoders = {B1, A1, A2, B2, A3, B3, A4, B4}
    // Se preserva el mismo orden/patron que la version anterior de este
    // archivo (no es A,B,A,B... simetrico -- motor1 arranca en B), solo
    // se actualizaron los valores de pin.
    // =========================================================
    constexpr uint8_t kEncoders[8] = {
        0,  // ENB_M1
        1,  // ENA_M1
        25, // ENA_M2
        24, // ENB_M2
        32, // ENA_M3
        31, // ENB_M3
        21, // ENA_M4
        20  // ENB_M4
    };

    // =========================================================
    // ELEVATOR (motor 5: IN1_M5/IN2_M5/PWM_M5 en el esquematico) --
    // confirmado por el equipo. No tiene ENA/ENB propio (usa los limit
    // switches en vez de encoder).
    // NOTA: Elevator.hpp/.cpp no referencian Pins:: actualmente (parece
    // hardcodear sus propios pines) -- estos valores no estan conectados
    // al comportamiento real todavia, eso es aparte de la duda de pines.
    // =========================================================

    // =========================================================
    // LIMIT SWITCH
    // El esquematico nuevo muestra DOS limit switches (antes solo habia
    // uno en este archivo). kLimitSwitch se mantiene con el mismo nombre
    // para no romper StateMachine.hpp (que ya lo usa); kLimitSwitch2 es
    // nuevo y todavia no esta conectado a ningun lado en el codigo.
    // Sin pull-up interna del Teensy -- pull-up externa en la PCB.
    // =========================================================
    constexpr uint8_t kLimitSwitch  = 7; // Limit1
    constexpr uint8_t kLimitSwitch2 = 6; // Limit2 // placeholder: no usado aun en el codigo, confirmar su rol (elevator top/bottom, sorter, etc.)

    // =========================================================
    // 74HC4067 MULTIPLEXERS (x2)
    // shared S0-S3, different SIG. Segun el equipo, AMBOS muxes son
    // para QTR (front/rear) -- el IR ya NO pasa por mux (ver seccion
    // IR mas abajo). kMuxSig2 solo esta declarado; instances.cpp hoy
    // unicamente instancia un Mux74HC4067 con kMuxSig (mux1), asi que
    // el segundo mux fisico esta disponible pero no usado en software
    // todavia.
    // =========================================================
    static constexpr uint8_t kMuxSig  = 26; // SIG_A0 -- mux1 (QTR front)
    static constexpr uint8_t kMuxSig2 = 22; // SIG_A1 -- mux2 (QTR rear)

    static constexpr uint8_t kMuxS0 = 27; // s0_MUX
    static constexpr uint8_t kMuxS1 = 28; // s1_MUX
    static constexpr uint8_t kMuxS2 = 29; // s2_MUX
    static constexpr uint8_t kMuxS3 = 30; // s3_MUX

    // =========================================================
    // QTR ARRAYS ON MUX1/MUX2
    // Numeros de canal dentro del mux (0-15), no pines fisicos del
    // Teensy -- el esquematico no los afecta directamente, sin cambios.
    // =========================================================
    static constexpr uint8_t kQtrFrontFirstCh = 0; // C0..C7
    static constexpr uint8_t kQtrRearFirstCh  = 8; // C8..C15

    // =========================================================
    // IR SENSORS -- GPIO DIRECTO (ya NO van por mux; instances.cpp ya
    // los trata como pines directos, ver comentario "IR directos" ahi).
    // L1-L4 y su posicion fisica (FL/FR/BL/BR) confirmados por el equipo.
    // =========================================================
    static constexpr uint8_t kIrChFL = 15; // L1
    static constexpr uint8_t kIrChFR = 14; // L2
    static constexpr uint8_t kIrChBL = 23; // L3
    static constexpr uint8_t kIrChBR = 41; // L4

    // =========================================================
    // TOF SENSORS ON I2C MUX (TCA9548A)
    // Numeros de canal del TCA9548A (0-7), no pines fisicos del
    // Teensy -- sin cambios. El bus I2C (SDA=18, SCL=19) es compartido
    // por BNO, PCA9685 y TCA9548A (confirmado en el esquematico).
    // =========================================================
    static constexpr uint8_t kToFchFR = 0; // Front Right
    static constexpr uint8_t kToFchFL = 1; // Front Left
    static constexpr uint8_t kToFchBL = 2; // Back Left / placeholder
    static constexpr uint8_t kToFchBR = 3; // Back Right / placeholder

    // =========================================================
    // SERVOS -- 6 conectores JST directos (SERVO1-SERVO6) confirmados
    // en la hoja SERVOS del esquematico completo, mas un PCA9685 (I2C)
    // aparte para hasta 7 servos adicionales.
    //
    // OJO / CONFLICTO A CONFIRMAR: en esa hoja, la señal de SERVO1 tiene
    // el mismo nombre de net que "INT" (pin 13) y la de SERVO5 el mismo
    // nombre que "RST" (pin 2) -- los mismos pines que antes el equipo
    // dijo que eran "extras del BNO, no conectados" (ver EXTRASBNO en
    // la hoja principal). Mismo nombre de net en el esquematico
    // normalmente = mismo nodo electrico, asi que lo mas probable es
    // que esos 2 pines EN REALIDAD esten manejando SERVO1/SERVO5, no
    // libres para el BNO. Se dejan mapeados aca con esa advertencia;
    // confirmar con el equipo antes de que el firmware los use para
    // otra cosa.
    //
    // SERVO6: no se pudo identificar su pin en el esquematico
    // compartido -- placeholder.
    //
    // Los 5 roles con nombre (Upper/Lower Intake, Separator, Benefit,
    // Holder) todavia no se pueden asignar a SERVO1-6 -- falta esa
    // relacion logica con el equipo.
    //
    // OJO -- CORRECCION: a diferencia de lo que decia antes esta nota,
    // ServoSystem.cpp SI usa estas constantes directamente como pines
    // GPIO reales (ver su constructor: Pins::kUpperIntakeServo, etc. ->
    // _pin[i], usados por la libreria Servo de Arduino, PWM directo, NO
    // el PCA9685). Con el valor actual (255) son pines INVALIDOS: si
    // este firmware llega a correr servos.begin() en hardware real tal
    // como esta, va a intentar usar el pin 255 -- no es solo un dato
    // pendiente, es un bug esperando a pasar. Hay que resolver esto
    // antes de subir cualquier build que llame servos.begin().
    // =========================================================
    constexpr uint8_t kServoPin1 = 13; // SERVO1 -- mismo net que "INT" (BNO), CONFIRMAR
    constexpr uint8_t kServoPin2 = 3;  // SERVO2
    constexpr uint8_t kServoPin3 = 4;  // SERVO3
    constexpr uint8_t kServoPin4 = 5;  // SERVO4
    constexpr uint8_t kServoPin5 = 2;  // SERVO5 -- mismo net que "RST" (BNO), CONFIRMAR
    // kServoPin6: placeholder, pin no identificado en el esquematico

    constexpr uint8_t kUpperIntakeServo = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kLowerIntakeServo = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kSeparatorServo   = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kBenefitServo     = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6
    constexpr uint8_t kHolderServo      = 255; // INVALIDO -- placeholder: falta mapear rol -> SERVO1-6

    // =========================================================
    // BNO055 -- RST/INT
    // Ver advertencia arriba (seccion SERVOS): estos mismos numeros de
    // pin coinciden con las señales de SERVO5/SERVO1 en el esquematico.
    // El equipo dijo antes que estos pines "no estan conectados" para
    // el BNO -- probablemente fueron reutilizados para los servos en
    // vez de quedar libres. PENDIENTE CONFIRMAR cual uso es el real
    // antes de que el firmware los use para cualquiera de los dos fines.
    // =========================================================
    static constexpr uint8_t kBnoRstReserved = 2;  // posible conflicto con SERVO5 -- confirmar
    static constexpr uint8_t kBnoIntReserved = 13; // posible conflicto con SERVO1 -- confirmar

    // NOTA: el equipo menciono "3 pines extra" ademas de los del BNO.
    // No se pudieron identificar sus numeros ni en el esquematico
    // inicial ni en el layout completo -- pendiente de confirmar antes
    // de asignarles un uso.

    // =========================================================
    // OPTIONAL / LEGACY ULTRASONICS
    // El esquematico nuevo no menciona ultrasonicos -- posiblemente
    // obsoleto / reemplazado por los IR directos y el ToF. Sin cambios,
    // se deja como estaba.
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
