#include "Drive.hpp"

// Actualized for Erick's new circuit board
//  Constructor

Drive::Drive()
    : bno_(),
    // Si no funciona, cambiar false o true, cambiar pines y despues false o true para 
     m1_ul_(33, 34, 8, true, UL_ENC_A, UL_ENC_B, diameter), //M1
     m2_ur_(37, 38, 10, true, LL_ENC_A, LL_ENC_B, diameter), //M3
     m3_ll_(35, 36, 9, true, UR_ENC_A, UR_ENC_B, diameter), //M2
     m4_lr_(40, 39, 11, true, LR_ENC_A, LR_ENC_B, diameter), //M4
      omni_(m1_ul_, m2_ur_, m3_ll_, m4_lr_),
      yawPid_(P, I, D, -kOmegaMax, +kOmegaMax),
      linePid_(0.00008f, 0.000005f, 0.0f, -kLinePidMax, +kLinePidMax)
{}

//  begin
void Drive::begin() {
    Serial.begin(115200);
    delay(500);

    analogWriteResolution(8);
    m1_ul_.begin(); m2_ur_.begin(); m3_ll_.begin(); m4_lr_.begin();

    Wire.begin();
    bno_.begin();
    bno_.update(); delay(50); bno_.update();

    yawPid_.setAngleWrapping(true);
    yawPid_.reset();

    m1_ul_.setPPR(475.0f);
    m2_ur_.setPPR(482.0f);
    m3_ll_.setPPR(495.0f);
    m4_lr_.setPPR(475.0f);

    targetYaw_      = bno_.getYaw();
    yawHoldEnabled_ = true;
    vxCmd_          = 0.0f;
    vyCmd_          = 0.0f;

    resetOdometry();
}

//  update - call always without delay 
void Drive::update() {
    const uint32_t now = millis();
    if (now - lastControl_ < kControlMs) return;
    lastControl_ = now;

    bno_.update();
    updateOdometry();

    // 1) Lógica busy (driveDistance / strafeDistance / turnTo)
    updateBusy(now);

    // 2) QTR line follow
    if (lineFollowEnabled_ && qtrFront_) {
        qtrFront_->update();

        if (qtrFront_->onLine(200)) {
            const float pos   = (float)qtrFront_->getPosition();
            const float error = pos - kLineCenter;

            float correction = 0.0f;
            if (fabsf(error) > kLineDeadband) {
                correction = linePid_.update(pos, kLineCenter);
            } else {
                linePid_.reset();
            }
            vxCmd_ = correction;
        }
    }

    // 3) Yaw PID
    const float yaw = bno_.getYaw();
    float omega = 0.0f;
    if      (manualOmegaEnabled_) omega = manualOmega_;
    else if (yawHoldEnabled_)     omega = yawPid_.update(yaw, targetYaw_);
    omega = clampf(omega, -kOmegaMax, +kOmegaMax);

    // 4) Movermotors
    if (rpmModeEnabled_) {
    m1_ul_.move((int)(fabsf(rpmSetUL_ + omega) / 60.0f * 255.0f),
                rpmSetUL_ + omega >= 0 ? DCMotor::Direction::FORWARD
                                        : DCMotor::Direction::BACKWARD);
    m2_ur_.move((int)(fabsf(rpmSetUR_ - omega) / 60.0f * 255.0f),
                rpmSetUR_ - omega >= 0 ? DCMotor::Direction::FORWARD
                                        : DCMotor::Direction::BACKWARD);
    m3_ll_.move((int)(fabsf(rpmSetLL_ + omega) / 60.0f * 255.0f),
                rpmSetLL_ + omega >= 0 ? DCMotor::Direction::FORWARD
                                        : DCMotor::Direction::BACKWARD);
    m4_lr_.move((int)(fabsf(rpmSetLR_ - omega) / 60.0f * 255.0f),
                rpmSetLR_ - omega >= 0 ? DCMotor::Direction::FORWARD
                                        : DCMotor::Direction::BACKWARD);
    } else {
    omni_.MoveXYW(vxCmd_, vyCmd_, -omega);
    }

    // 4) Debug 10 Hz
    if (now - lastPrint_ >= kPrintMs) {
        lastPrint_ = now;
        Serial.printf("x:%.3f y:%.3f dist:%.3f yaw:%.1f\n",
            ekf_.getX(), ekf_.getY(), ekf_.getDist(), rad2deg(yaw));
    }
}

//  updateOdometry — deltas from encoder → EKF

void Drive::updateOdometry() {
    const float d1 = m1_ul_.getDistanceMeters();
    const float d2 = m2_ur_.getDistanceMeters();
    const float d3 = m3_ll_.getDistanceMeters();
    const float d4 = m4_lr_.getDistanceMeters();

    const float dd1 = d1 - prevD1_;
    const float dd2 = d2 - prevD2_;
    const float dd3 = d3 - prevD3_;
    const float dd4 = d4 - prevD4_;

    prevD1_ = d1; prevD2_ = d2;
    prevD3_ = d3; prevD4_ = d4;

    // Delta distance → RPM equivalent for the EKF
    const float dt   = kControlMs / 1000.0f;
    const float circ = M_PI * diameter;  // circunferencia = π × diámetro

    const float rpmUL = (dd1 / circ) * 60.0f / dt;
    const float rpmUR = (dd2 / circ) * 60.0f / dt;
    const float rpmLL = (dd3 / circ) * 60.0f / dt;
    const float rpmLR = (dd4 / circ) * 60.0f / dt;

    ekf_.step(dt, rpmUL, rpmUR, rpmLL, rpmLR, bno_.getYaw());
}

//  resetOdometry
void Drive::resetOdometry() {
    ekf_.resetPose();
    prevD1_ = prevD2_ = prevD3_ = prevD4_ = 0.0f;
    m1_ul_.resetEncoder();
    m2_ur_.resetEncoder();
    m3_ll_.resetEncoder();
    m4_lr_.resetEncoder();
}

//  updateBusy — gestiona driveDistance / strafeDistance / turnTo
void Drive::updateBusy(uint32_t now) {
    if (!busy_) return;

    bool done = false;

    if (distTarget_ < 0.0f) {
        // Modo giro: espera error de yaw < 2°
        const float err = fabsf(yawPid_.getError());
        if (err < deg2rad(2.0f) || (now - busyStart_) > busyTimeout_)
            done = true;
    } else {
        // Modo distancia: usa EKF
        const float traveled = ekf_.getDist();
        if (traveled >= distTarget_ || (now - busyStart_) > busyTimeout_)
            done = true;
    }

    if (done) {
        busy_ = false;
        stop();
    }
}

//  driveDistance
void Drive::driveDistance(float meters, float speed) {
    resetOdometry();
    distTarget_  = fabsf(meters);
    busyVx_      = (meters >= 0.0f) ? +speed : -speed;
    busyVy_      = 0.0f;
    busy_        = true;
    busyStart_   = millis();
    busyTimeout_ = (uint32_t)(fabsf(meters) / speed * 1000.0f * 2.5f);

    vxCmd_ = busyVx_;
    vyCmd_ = 0.0f;
    holdYaw(true);
}

//  strafeDistance
void Drive::strafeDistance(float meters, float speed) {
    resetOdometry();
    distTarget_  = fabsf(meters);
    busyVx_      = 0.0f;
    busyVy_      = (meters >= 0.0f) ? +speed : -speed;
    busy_        = true;
    busyStart_   = millis();
    busyTimeout_ = (uint32_t)(fabsf(meters) / speed * 1000.0f * 2.5f);

    vxCmd_ = 0.0f;
    vyCmd_ = busyVy_;
    holdYaw(true);
}

//  turnTo
void Drive::turnTo(float targetDeg, uint32_t timeoutMs) {
    stop();
    setTargetYaw(deg2rad(targetDeg));
    distTarget_  = -1.0f;
    busy_        = true;
    busyStart_   = millis();
    busyTimeout_ = timeoutMs;
    holdYaw(true);
}

//  QTR
void Drive::attachQTR(QTR& qtr) {
    qtrFront_ = &qtr;
    linePid_.reset();
}

void Drive::followLine(float vySpeed) {
    if (!qtrFront_) return;
    lineFollowEnabled_ = true;
    lineSpeedVy_       = vySpeed;
    vyCmd_             = vySpeed;
    vxCmd_             = 0.0f;
    linePid_.reset();
    holdYaw(true);
}

void Drive::stopLineFollow() {
    lineFollowEnabled_ = false;
    linePid_.reset();
    stop();
}

//  Basic movement
void Drive::forward(float speed)      { vxCmd_ = +speed; vyCmd_ = 0.0f;  }
void Drive::backward(float speed)     { vxCmd_ = -speed; vyCmd_ = 0.0f;  }
void Drive::right(float speed)        { vxCmd_ = 0.0f;   vyCmd_ = -speed; }
void Drive::left(float speed)         { vxCmd_ = 0.0f;   vyCmd_ = +speed; }
void Drive::diagonalLeft(float speed) { vxCmd_ = +speed; vyCmd_ = +speed; }
void Drive::stop()                    { vxCmd_ = 0.0f;   vyCmd_ = 0.0f;  
                                        rpmModeEnabled_ = false; }
void Drive::setTranslation(float vx, float vy) { vxCmd_ = vx; vyCmd_ = vy; }

void Drive::brake() {
    m1_ul_.stop(); m2_ur_.stop(); m3_ll_.stop(); m4_lr_.stop();
}

//  Yaw hold
void Drive::holdYaw(bool enable) {
    yawHoldEnabled_ = enable;
    if (enable) {
        targetYaw_ = bno_.getYaw();
        yawPid_.resetToMeasurement(targetYaw_, targetYaw_);
    }
}

void Drive::setTargetYaw(float yawRad) {
    targetYaw_ = yawRad;
    yawPid_.resetToMeasurement(bno_.getYaw(), yawRad);
}

float Drive::getYaw() const {
    return const_cast<Drive*>(this)->bno_.getYaw();
}

// Manuel Omega
void Drive::setManualOmega(float omegaRadS) {
    manualOmegaEnabled_ = true;
    manualOmega_        = omegaRadS;
}

void Drive::clearManualOmega() {
    manualOmegaEnabled_ = false;
    manualOmega_        = 0.0f;
    yawPid_.resetToMeasurement(bno_.getYaw(), targetYaw_);
}

//  Helpers
float Drive::rad2deg(float r) { return r * (180.0f / M_PI); }
float Drive::deg2rad(float d) { return d * (M_PI / 180.0f); }
float Drive::clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}
float Drive::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

//  testKinematics
void Drive::testKinematics(float v, uint32_t T) {
    holdYaw(false);
    Serial.println("Forward");  omni_.MoveXYW(+v,  0,  0); delay(T);
    Serial.println("Backward"); omni_.MoveXYW(-v,  0,  0); delay(T);
    Serial.println("Right");    omni_.MoveXYW( 0, +v,  0); delay(T);
    Serial.println("Left");     omni_.MoveXYW( 0, -v,  0); delay(T);
    Serial.println("Diag++");   omni_.MoveXYW(+v, +v,  0); delay(T);
    Serial.println("Diag--");   omni_.MoveXYW(-v, -v,  0); delay(T);
    Serial.println("Stop");     omni_.Stop();                delay(1200);
}

//  beginNoBNO / updateNoBNO
void Drive::beginNoBNO() {
    Serial.begin(115200);
    delay(500);

    analogWriteResolution(8);
    m1_ul_.begin(); m2_ur_.begin(); m3_ll_.begin(); m4_lr_.begin();

    m1_ul_.setPPR(475.0f);
    m2_ur_.setPPR(482.0f);
    m3_ll_.setPPR(495.0f);
    m4_lr_.setPPR(475.0f);

    vxCmd_ = 0.0f;
    vyCmd_ = 0.0f;

    resetOdometry();
}

void Drive::updateNoBNO() {
    const uint32_t now = millis();
    if (now - lastControl_ < kControlMs) return;
    lastControl_ = now;

    // Simple odometry without EKF
    const float d1 = m1_ul_.getDistanceMeters();
    const float d2 = m2_ur_.getDistanceMeters();
    const float d3 = m3_ll_.getDistanceMeters();
    const float d4 = m4_lr_.getDistanceMeters();

    const float dd1 = d1 - prevD1_;
    const float dd2 = d2 - prevD2_;
    const float dd3 = d3 - prevD3_;
    const float dd4 = d4 - prevD4_;

    prevD1_ = d1; prevD2_ = d2;
    prevD3_ = d3; prevD4_ = d4;

    constexpr float k = 0.7071f * 0.25f;
    const float odoX = k * (-dd1 + dd2 + dd3 - dd4);
    const float odoY = k * ( dd1 + dd2 - dd3 - dd4);

    omni_.MoveXYW(vxCmd_, vyCmd_, 0.0f);

    if (now - lastPrint_ >= kPrintMs) {
        lastPrint_ = now;
        Serial.print("odoX: "); Serial.print(odoX, 4);
        Serial.print("  odoY: "); Serial.println(odoY, 4);
    }
}

void Drive::setRPMs(float ul, float ur, float ll, float lr) {
    rpmSetUL_ = ul;
    rpmSetUR_ = ur;
    rpmSetLL_ = ll;
    rpmSetLR_ = lr;
    rpmModeEnabled_ = true;
}