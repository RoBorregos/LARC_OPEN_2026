
/*
    Comando  para testDriveStateMachineTest.cpp
    @concept: Se recrea la maquina de estados original ahora resguadada en "AntiqueStateMachine"
        pero se esta usando con las siguientes modificaciones:
        - Se esta usando Drive (just yaw hold:true) para control.
        - Se omiten por el momento lecturas de VLX estan guardadas ObstacleDetected: false.
        - Se omite la lectura de uno de los 

    pio run -e drive_statemachine_test -t upload -t monitor
*/
#include <Arduino.h>
#include "DriveStateMachineTest.hpp"
#include "robot/instances/instances.hpp"
#include "constants.h"
#include "Vision.hpp"

namespace
{
    static constexpr float kVelocity = 0.30f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    // Corrección lateral para el retroceso en LOOKFORCORNER / BEANSGOBACK.
    // Se probo con PIDController (PID_v1) pero el Ki metia un ciclo limite
    // lento (se pasa de largo, tarda en corregir al otro lado). Ya no hace
    // falta: el ruido original que motivo el PID venia de lecturas sucias
    // del QTR (arreglado con el filtro de mediana en QTR::update()), asi
    // que un P puro manual -- calculado cada loop, sin el SampleTime de la
    // libreria -- alcanza. kCornerKp se afina empiricamente en campo.
    static constexpr float kCornerSetpoint = 2900.0f;
    static constexpr float kCornerKp = 0.00012f;
    // El swing violento que forzo bajar esto a 0.12 probablemente venia
    // del signo invertido en LOOKFORCORNER (ya corregido): corregia hacia
    // el lado equivocado, lo cual generaria justo ese tipo de oscilacion.
    // Con el signo ya bien, 0.12 resulto insuficiente para recuperar de
    // un error grande (lPos se quedaba clavado sin volver al setpoint) ->
    // se sube.
    //
    // OJO: kCornerKp/kCornerCorrMax se afinaron en campo cuando la
    // correccion iba en vy (strafe, ver setTranslation(-0.20f,
    // -cornerCorrFiltered) mas abajo). Ahora que vx/vy se intercambiaron
    // para que vx=correccion/vy=velocidad base (ver handleLookForCornerState),
    // la respuesta mecanica del chasis puede ser distinta en ese eje ->
    // revisar si siguen siendo apropiadas o hay que volver a afinarlas.
    static constexpr float kCornerCorrMax = 0.20f;

    // handleBEANS usa 2200 (literal, ver mas abajo) como setpoint en vez de
    // este mismo valor -- son el mismo QTR fisico, no deberia haber dos
    // "centros" distintos. Dejado asi por ahora; revisar/unificar despues.

    // Corrección lateral (izq/der) para el retroceso en LOOKFORLINEBACKWARDS,
    // usando qtrRear -- mismo patron P + filtro paso-bajo que
    // LOOKFORCORNER/BEANS, pero aqui la corrección va en vy (no vx) porque
    // el desplazamiento principal de este estado es hacia atras (vx), no
    // lateral (al reves que Corner/BEANS). kRearSetpoint usa el centro
    // matematico (3000, indice 3 de 7 sensores) en vez de un valor afinado
    // en campo como kCornerSetpoint -- sin afinar todavia. El signo de la
    // correccion tampoco esta verificado: a diferencia de qtrFront (C6=
    // adelante, C0=atras, confirmado en campo), no se sabe que canal de
    // qtrRear (C8-C14) corresponde a izquierda/derecha. Si el robot se
    // aleja de la linea en vez de corregir, invertir el signo en
    // handleLookForLineBackWards.
    static constexpr float kRearSetpoint  = 3000.0f;
    static constexpr float kRearKp        = 0.00012f;
    static constexpr float kRearCorrMax   = 0.20f;
    static constexpr float kRearCorrAlpha = 0.15f;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs = 4500;    // Time to ignore IR's at the START point
    static constexpr uint32_t kClearDelayMs = 1500;  //6500;          // Tiempo para cambiar nuevamente a Forward
    static constexpr uint32_t kNoObstacleToCornerMs = 3000; // Time without obstacle to go forward and LOOKFORLINE -> tal vez disminuir
    static constexpr uint32_t kCornerDeployWazitMs = 1800;

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTofTargetMm = 120;   // distancia deseada al árbol
    static constexpr uint16_t kTofHardStopMm = 105; // stop de seguridad

    static constexpr float kTofMinSpeed = 0.10f;
    static constexpr float kTofMaxSpeed = 0.35f;
    static constexpr float kObstacleDistanceCm = 20.0f;


    static constexpr float kDistKp = 0.0012f;
    static constexpr float kDistKi = 0.0f;
    static constexpr float kDistKd = 0.00015f;

    // Limit switch aun no cableado en el hardware actual: sin esto, INPUT_PULLUP
    // deja el pin flotando en HIGH y el codigo lo lee como "presionado" siempre,
    // trabando handleStartState en el loop de emergencia (case 1). Poner en
    // true en cuanto el switch este conectado.
    static constexpr bool kLimitSwitchConnected = false;

    const __FlashStringHelper *mainStateName(DriveTestSTATES state)
    {
        switch (state)
        {
        case DriveTestSTATES::START:
            return F(""); // F("START ♡ ♡ ♡");
        case DriveTestSTATES::POOL:
            return F(""); // F("POOL");
        case DriveTestSTATES::LOOKFORLINE:
            return F(""); // F("LOOKFORLINE");
        case DriveTestSTATES::LOOKFORCORNER:
            return F(""); // F("LOOKFORCORNER");
        case DriveTestSTATES::BEANS:
            return F(""); // F("BEANS");
        case DriveTestSTATES::BEANSGOBACK:
            return F(""); // F("BEANSGOBACK");
        case DriveTestSTATES::POOLSGOBACK:
            return F(""); // F("POOLSGOBACK");
        case DriveTestSTATES::LOOKFORLINEBACKWARDS:
            return F(""); // F("LOOKFORLINEBACKWARDS");
        case DriveTestSTATES::BENEFITSSTARTCORNER:
            return F(""); // F("BENEFITSSTARTCORNER");
        case DriveTestSTATES::BENEFITS:
            return F(""); // F("BENEFITS");
        case DriveTestSTATES::STOP:
            return F(""); // F("STOP ♡ ♡ ♡ ♡ ♡");
        default:
            return F(""); // F("DEFAULT");
        }
    }

    const __FlashStringHelper *poolStateName(PoolSubState state)
    {
        switch (state)
        {
        case PoolSubState::FORWARD:
            return F(""); // F("FORWARD");
        case PoolSubState::AVOID_LEFT:
            return F(""); // F("AVOID_LEFT");
        case PoolSubState::AVOID_RIGHT:
            return F(""); // F("AVOID_RIGHT");
        default:
            return F(""); // F("DEFAULT");
        }
    }
}

DriveStateMachineTest::DriveStateMachineTest()
{
}

void DriveStateMachineTest::begin()
{

    currentState = DriveTestSTATES::LOOKFORLINE; // always in START
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = millis();
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    visionLeft = 0;
    visionRight = 0;

    //Elevator

    pinMode(limitSwitch, INPUT_PULLUP);

    vision.begin();
    vision.requestStatus();

    Wire.begin();
    Wire.setClock(400000);
    i2cMux.begin();

    bool okL = tofLeft.begin();
    bool okR = tofRight.begin();

    Serial.print("tofLeft init: ");  Serial.println(okL ? "OK" : "FAIL");
    Serial.print("tofRight init: "); Serial.println(okR ? "OK" : "FAIL");

    //QTR
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);   // FRONT qtr

    tofLeft.setMaxRange(600);
    tofRight.setMaxRange(600);

    tofLeft.setUpdateInterval(30);
    tofRight.setUpdateInterval(30);

    ir.begin();
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    // Reemplaza a odomMove_.begin()/setCommandTimeout/resetPose/captureCurrentYawTarget:
    // LARC (Drive) ya arranca el BNO085 (Wire2), engancha el PID de yaw-hold
    // y resetea su propia odometría EKF internamente.
    LARC.begin();
    LARC.holdYaw(true);
}

void DriveStateMachineTest::update()
{
    ir.update();
    qtrFront.update();
    qtrRear.update();
    vision.update();
    const uint32_t now = millis();
    startStateTime();

    const int linePos = qtrFront.getPosition(); // Para el PID → más suave
    const bool onLine = qtrFront.onLine();
    const float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    const float vx = -lineCorr;

    const bool FL = ir.getState(IRLine::FL);
    const bool FR = ir.getState(IRLine::FR);
    const bool BL = ir.getState(IRLine::BL);
    const bool BR = ir.getState(IRLine::BR);

// =========== Print to debug =============
    //ir.debugPrint();
    //qtrFront.debugPrint();
    //Serial.print("linePos: ");
    //Serial.println(linePos); // To know the value for the center of the qtr


    static uint32_t debugPrintMs = 0;
    if ((now - debugPrintMs) >= 100)
    {
    debugPrintMs = now;

    // Yaw (LARC/BNO085) -- sin odometría de posición, solo el heading que usa el yaw-hold
    Serial.print(F("DriveStateMachineTest"));
    Serial.print(F(" ❤ Yaw❤ | Deg:")); Serial.print(LARC.getYaw() * 180.0f / PI, 1);

    // Estado actual
    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState); Serial.print(")");
    Serial.print(F(" PS:")); Serial.print((int)poolState);
    Serial.print(F(" AS:")); Serial.print(action_stage);
    Serial.print(F(" LSW:")); Serial.print(digitalRead(limitSwitch));

    // ToF
    Serial.print(F(" ❤ Tof❤ |"));  //Serial.print(tofLeft.getDistanceCm(),  1);
    //Serial.print(F(" TR:"));    Serial.print(tofRight.getDistanceCm(), 1);
    //Serial.print(F(" vL:"));    Serial.print(tofLeft.isValid());
    //Serial.print(F(" vR:"));    Serial.print(tofRight.isValid());
    Serial.print(F(" TL:")); Serial.print(tofLeft.getDistanceCm(), 0);
    Serial.print(F("cm vL:")); Serial.print(tofLeft.isValid() ? "OK" : "NO");
    Serial.print(F(" TR:")); Serial.print(tofRight.getDistanceCm(), 0);
    Serial.print(F("cm vR:")); Serial.print(tofRight.isValid() ? "OK" : "NO");

    // Obstáculo
    //Serial.print(F(" | OBS:")); Serial.print(obstacle);
    //Serial.print(F(" OL:"));    Serial.print(obstacleLeftNow);
    //Serial.print(F(" OR:"));    Serial.print(obstacleRightNow);

    // IR
    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(FL);
    Serial.print(F(" FR:"));   Serial.print(FR);
    Serial.print(F(" BL:"));   Serial.print(BL);
    Serial.print(F(" BR:"));   Serial.print(BR);

    // Línea
    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(onLine);
    Serial.print(F(" lPos:"));  Serial.print(qtrFront.getPosition());
    Serial.print(F(" vx:")); Serial.print(vx);

    // Diagnostico temporal: raw/norm crudos del QTR frontal, sensor por
    // sensor, para confirmar si hay contraste real llegando (calibracion/
    // wiring) o si onLine() nunca dispara porque el sensor no ve la linea.
    const uint16_t* qtrRaw  = qtrFront.getRaw();
    const uint16_t* qtrNorm = qtrFront.getNorm();
    Serial.print(F(" | raw:"));
    for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(qtrRaw[i]); Serial.print(','); }
    Serial.print(F(" norm:"));
    for (uint8_t i = 0; i < QTR::N; i++) { Serial.print(qtrNorm[i]); Serial.print(','); }

    Serial.println();
    }



    const bool frontLeftDetectedLine = FL; // Also used for corner
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BR;
    const bool backRightDetectedLine = BL;
    const bool frontDetectedLine = (FL || FR); // Hacer que con el qtr tambien detecte linea
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    // DESPUÉS
    static constexpr uint32_t kTofWarmupMs = 500;
    static uint32_t tofReadyTimestamp = 0;
    if (tofReadyTimestamp == 0 && (tofLeft.isValid() || tofRight.isValid()))
        tofReadyTimestamp = now;
    const bool tofReady = tofReadyTimestamp != 0 &&
                        (now - tofReadyTimestamp) > kTofWarmupMs;

    const bool obstacleLeftNow  = false;/*tofReady
                            && tofLeft.isValid()
                            && tofLeft.getDistanceCm()  < kObstacleDistanceCm;*/

    const bool obstacleRightNow = false;/*tofReady
                            && tofRight.isValid()
                            && tofRight.getDistanceCm() < kObstacleDistanceCm;*/

    static bool obstacleLatched = false;
    static uint32_t obstacleClearStartMs  = 0;
    static uint32_t obstacleDetectStartMs = 0;          // ← nuevo
    static constexpr uint32_t kObstacleReleaseMs  = 400; // ← subido de 200 a 400
    static constexpr uint32_t kObstacleConfirmMs  = 0; //50;  // ← nuevo: ms consecutivos para activar

    if (!obstacleLatched)
    {
    if (obstacleLeftNow || obstacleRightNow)
    {
    if (obstacleDetectStartMs == 0)
        obstacleDetectStartMs = now;

    if ((now - obstacleDetectStartMs) >= kObstacleConfirmMs)
    {
        obstacleLatched       = true;
        obstacleClearStartMs  = 0;
        obstacleDetectStartMs = 0;
    }
    }
    else
    {
    obstacleDetectStartMs = 0; // reset si deja de verse
    }
    }
    else
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleClearStartMs = 0;
        }
        else
        {
            if (obstacleClearStartMs == 0)
                obstacleClearStartMs = now;

            if ((now - obstacleClearStartMs) >= kObstacleReleaseMs)
            {
                obstacleLatched = false;
                obstacleClearStartMs = 0;
            }
        }
    }
    const bool obstacle = obstacleLatched;

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
        handlePOOLSGoBackState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case DriveTestSTATES::LOOKFORLINEBACKWARDS:
        handleLookForLineBackWards(now, backDetected, backLeftDetectedLine, backRightDetectedLine);
        break;

    case DriveTestSTATES::BENEFITSSTARTCORNER:
        handleBenefitsStartCorner(now, frontLeftDetectedLine, vx, onLine);
        break;

    case DriveTestSTATES::BENEFITS:
        handleBenefits(now, backLeftDetectedLine, vx, onLine);
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

    // Reset corrección lateral LOOKFORLINE
    lfCorrecting        = false;
    lfCorrectionDir     = 0;
    lfCorrectionStartMs = 0;

    // Reset filtro paso-bajo del corr lateral de LOOKFORCORNER / LOOKFORLINEBACKWARDS
    cornerCorrFiltered = 0.0f;
    rearCorrFiltered = 0.0f;

    // Filtro de mediana del QTR: sin esto, las primeras
    // lecturas del nuevo estado quedan mezcladas con las últimas del
    // estado anterior (que pudo estar viendo una parte de la línea muy
    // distinta), retrasando la corrección real justo al entrar.
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

void DriveStateMachineTest::handleStartState(uint32_t now, bool backDetected)
{
    vision.stop();

    const bool limitPressed = kLimitSwitchConnected && (digitalRead(limitSwitch) == HIGH); // ==HIGH

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
    // ── Subir por 9000 ms ────────────────────────────────────────────────
    case 0:
        if (limitPressed)
        {
            // Limit presionado durante la subida → interrumpir y bajar
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
                // Subida completa → pasar al elevador stop
                action_start_time = now;
                action_stage = 4;
            }
        }
        break;

    // ── Bajar mientras limit esté presionado ─────────────────────────────
    case 1:
        elevator.ElevatorPosition(1);
        LARC.stop();

        if (!limitPressed)
        {
            // Limit suelto → esperar 2000 ms antes de reintentar subida
            action_start_time = now;
            action_stage = 2;
        }
        break;

    // ── Esperar 2000 ms con elevador parado ──────────────────────────────
    case 2:
        elevator.ElevatorPosition(0);
        LARC.stop();

        if ((now - action_start_time) >= 2000)
        {
            // Reintentar subida desde cero
            action_start_time = now;
            action_stage = 0;
        }
        break;

    // ── Subida completa: elevador stop 3000 ms ───────────────────────────
    case 4:
        elevator.ElevatorPosition(0);
        LARC.stop();
        if ((now - action_start_time) >= 1500)
        {
            action_start_time = now;
            action_stage = 5;
        }
        break;

    // ── Avanzar y transicionar a POOL ────────────────────────────────────
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
        // Si el obstáculo se acerca demasiado, retroceder
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid()  && distL < 12.0f) ||
                              (tofRight.isValid() && distR < 12.0f);

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
        // Si el obstáculo se acerca demasiado, retroceder
        const float distL = tofLeft.getDistanceCm();
        const float distR = tofRight.getDistanceCm();
        const bool tooClose = (tofLeft.isValid()  && distL < 10.0f) ||
                              (tofRight.isValid() && distR < 10.0f);

        if (tooClose)
        {
            LARC.backward(0.30f);
            break;
        }

        LARC.right(0.30f);

        const bool justEntered = (now - poolStateStartMs) < 150;

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
    // ── case 0: retroceder 400 ms ─────────────────────────────────────────
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

    // ── case 1: avanzar 400 ms ────────────────────────────────────────────
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

    // ── case 2: stop 400 ms ───────────────────────────────────────────────
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

    // ── case 3: búsqueda normal ───────────────────────────────────────────
    static constexpr uint32_t kBorderCorrectMs = 150;
    static constexpr float    kTofBorderCm     = 15.0f;

    const bool realBorderLeft  = leftDetected  && tofLeft.isValid()  && tofLeft.getDistanceCm()  > kTofBorderCm;
    const bool realBorderRight = rightDetected && tofRight.isValid() && tofRight.getDistanceCm() > kTofBorderCm;

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

    static constexpr uint32_t kCornerStopMs = 8200;//1200; //Para que vision empiece
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

        // Sin esto, en cuanto se pierde la línea qtrFront.getPosition() se
        // queda congelada en la última lectura (ver QTR::update(), sum==0
        // mantiene position anterior) y el P sigue empujando con ese error
        // viejo -- normalmente hacia el borde por donde se perdió -- sin
        // ninguna lectura fresca que lo traiga de vuelta. Resultado: sigue
        // derivando para ese lado en vez de corregir. Al no ver línea,
        // se congela el target en 0 hasta reencontrarla.
        float corrTarget = 0.0f;
        if (onLine)
        {
            const float error = kCornerSetpoint - qtrFront.getPosition();
            corrTarget = constrain(error * kCornerKp, -kCornerCorrMax, kCornerCorrMax);
        }

        // Filtro paso-bajo: lPos trae ruido (EMI de los motores nuevos, ver
        // debug con raw/norm) que hace saltar corrTarget entre +max y -max
        // en menos de 1ms. Sin suavizar, ese ruido pasa directo al comando
        // y el robot vibra en vez de corregir suave hacia el centro. alpha
        // bajo = mas lento pero rechaza mejor el ruido.
        static constexpr float kCornerCorrAlpha = 0.15f;
        cornerCorrFiltered += (corrTarget - cornerCorrFiltered) * kCornerCorrAlpha;

        //Serial.print(F("[LOOKFORCORNER] onLine:")); Serial.print(onLine);
        //Serial.print(F(" lPos:")); Serial.print(qtrFront.getPosition());
        //Serial.print(F(" corrTarget:")); Serial.print(corrTarget, 4);
        //Serial.print(F(" corrFilt:")); Serial.println(cornerCorrFiltered, 4);

        // vx = corrección QTR (adelante/atrás), vy = velocidad base hacia
        // la izquierda -- ver Drive::forward()/left() en Drive.cpp
        // (vx=adelante/atras, vy=izq/der). Signo confirmado en campo: C6
        // (indice 6, position alto/cerca de 6000) es el sensor mas
        // adelantado del arreglo, C0 (position bajo) el mas trasero. Si
        // la linea esta hacia C6, position > setpoint -> error negativo
        // -> corrFiltered negativo -> con el "-" de abajo, vx>0 (corrige
        // hacia adelante, alcanzando la linea). Sin el "-", corregiria al
        // reves.
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

        /*
        if (!onLine)
        {
            if (action_start_time == 0)
                action_start_time = now;

            LARC.backward(0.30f);

            if ((now - action_start_time) >= kLostLineTimeoutMs)
            {
                vision.stop();
                setState(DriveTestSTATES::POOLSGOBACK);
            }

            return;
        }*/

        action_start_time = 0;

        // 2200 en vez de kCornerSetpoint (2900) -- mismo QTR fisico que
        // LOOKFORCORNER, ver nota junto a kCornerSetpoint sobre unificar
        // esto despues.
        float corr = 0.0f;
        if (onLine)
        {
            const int error = 2200 - qtrFront.getPosition();
            corr = constrain(error * 0.0003f, -0.3f, 0.3f);
        }

        // vx = corrección QTR (adelante/atrás), vy = velocidad base hacia
        // la derecha -- mismo eje y mismo signo que handleLookForCornerState
        // (C6=sensor mas adelantado, C0=mas trasero, confirmado en campo).
        LARC.setTranslation(-corr, -kVelocity);

        break;
    }

    case 1:
    {
        LARC.stop();
        if ((now - action_start_time) >= 1000)
        {
            action_start_time = 0;
            setState(DriveTestSTATES::BEANSGOBACK);
        }
        break;
    }
    }
}

void DriveStateMachineTest::handleBEANSGoBackState(uint32_t now, bool BL, bool BR, bool FL)
{
    // No bajar el elevador :: 

    switch (action_stage)
    {
    // ── case 0: retroceder hasta encontrar BR o BL ──────────────────────
    case 0:
        vision.stop();
        vision.clearErrors();
        elevator.ElevatorPosition(0); //zero for stop

        if (BR || BL)
        {
            LARC.stop();
            action_stage = 1;
            return;
        }

        LARC.backward(0.30f);
        return;

    // ── case 1: LARC.left hasta encontrar FL ────────────────────────────
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

    // ── case 2: detenido ─────────────────────────────────────────────────
    case 2:
        elevator.ElevatorPosition(0);
        LARC.stop();
        return;
    }
}

void DriveStateMachineTest::handlePOOLSGoBackState(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected)
{
switch (poolState)
{
case PoolSubState::FORWARD:
{
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

void DriveStateMachineTest::handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected)
{
    if (backDetected)
    {
        setState(DriveTestSTATES::BENEFITSSTARTCORNER);
        return;
    }

    // Mismo patron que LOOKFORCORNER: si se pierde la línea, congelar el
    // target en 0 en vez de arrastrar qtrRear.getPosition() congelada
    // (ver QTR::update(), sum==0 mantiene la posición anterior).
    float corrTarget = 0.0f;
    if (qtrRear.onLine())
    {
        const float error = kRearSetpoint - qtrRear.getPosition();
        corrTarget = constrain(error * kRearKp, -kRearCorrMax, kRearCorrMax);
    }

    rearCorrFiltered += (corrTarget - rearCorrFiltered) * kRearCorrAlpha;

    // vx = velocidad base hacia atrás (fija), vy = corrección qtrRear
    // (izq/der) -- al revés que LOOKFORCORNER/BEANS, porque aquí el
    // desplazamiento principal es hacia atrás, no lateral. Signo de
    // rearCorrFiltered SIN VERIFICAR en hardware todavía (a diferencia de
    // qtrFront/C6-C0, no se sabe que canal de qtrRear -C8-C14- es
    // izquierda/derecha). Si el robot se aleja de la línea en vez de
    // corregir, invertir el signo aquí.
    LARC.setTranslation(-kBaseSpeed, rearCorrFiltered);
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

        if (!onLine)
        {
            LARC.left(kBaseSpeed); // <- Provicionial LARC.stop();
            return;
        }

        LARC.setTranslation(vx, 0.30f);

        break;
    }

    case 1:
    {
        LARC.brake();
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
            LARC.brake();
            action_start_time = now;
            action_stage = 2;
        }
        break;
    }

    case 1:
    {
        LARC.setTranslation(vx, -0.30f);

        // Here goes the rutine
        if (cornerRIGHTDetected)
        {
            action_stage = 2;
        }
        break;
    }

    case 2:
    {
        LARC.brake();

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
    LARC.brake();
}

void DriveStateMachineTest::updateControl()
{
    LARC.update();
}
