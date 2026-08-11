#include "testOdometry.hpp"

OdomMovement* OdomMovement::instance_ = nullptr;

OdomMovement::OdomMovement()
    : bno_(),

      encUL_A_(Pins::kEncoders[1]),
      encUL_B_(Pins::kEncoders[0]),
      pwmUL_(Pins::kPwmPin[0]),
      inUL1_(Pins::kUpperMotors[0]),
      inUL2_(Pins::kUpperMotors[1]),

      encUR_A_(Pins::kEncoders[2]),
      encUR_B_(Pins::kEncoders[3]),
      pwmUR_(Pins::kPwmPin[1]),
      inUR1_(Pins::kUpperMotors[2]),
      inUR2_(Pins::kUpperMotors[3]),

      encLL_A_(Pins::kEncoders[4]),
      encLL_B_(Pins::kEncoders[5]),
      pwmLL_(Pins::kPwmPin[2]),
      inLL1_(Pins::kLowerMotors[0]),
      inLL2_(Pins::kLowerMotors[1]),

      encLR_A_(Pins::kEncoders[6]),
      encLR_B_(Pins::kEncoders[7]),
      pwmLR_(Pins::kPwmPin[3]),
      inLR1_(Pins::kLowerMotors[2]),
      inLR2_(Pins::kLowerMotors[3]),

      UL_{pwmUL_, inUL1_, inUL2_, {}, 0, 0, false, 482.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, false},
      UR_{pwmUR_, inUR1_, inUR2_, {}, 0, 0, false, 475.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, true},
      LL_{pwmLL_, inLL1_, inLL2_, {}, 0, 0, false, 495.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, false},
      LR_{pwmLR_, inLR1_, inLR2_, {}, 0, 0, false, 486.0f, kKp, kKi, kKd, 0.0f, 0.0f, 0.0f, true},

      ticksLL_count_(0),
      prevTicksLL_(0),

      yawTarget_(0.0f),
      yawIntegral_(0.0f),
      yawPrevErr_(0.0f),
      yawNow_(0.0f),

      ekf_x_(0.0f),
      ekf_y_(0.0f),
      ekf_th_(0.0f),

      R_theta_(0.02f),
      lastCycleMs_(0)
{
    P_[0][0] = 0.01f; P_[0][1] = 0.0f;  P_[0][2] = 0.0f;
    P_[1][0] = 0.0f;  P_[1][1] = 0.01f; P_[1][2] = 0.0f;
    P_[2][0] = 0.0f;  P_[2][1] = 0.0f;  P_[2][2] = 0.05f;

    Q_[0][0] = 0.0005f; Q_[0][1] = 0.0f;    Q_[0][2] = 0.0f;
    Q_[1][0] = 0.0f;    Q_[1][1] = 0.0005f; Q_[1][2] = 0.0f;
    Q_[2][0] = 0.0f;    Q_[2][1] = 0.0f;    Q_[2][2] = 0.003f;
}

float OdomMovement::wrapPi(float a)
{
    while (a >  PI) a -= 2 * PI;
    while (a < -PI) a += 2 * PI;
    return a;
}

void OdomMovement::pushPeriod(Motor& m, unsigned long p)
{
    m.period_buf[m.period_idx] = p;
    m.period_idx = (m.period_idx + 1) % FILTER_SIZE;
    m.got_pulse = true;
}

float OdomMovement::yawPidStep(float yawMeasured, float dt)
{
    float error = wrapPi(yawMeasured - yawTarget_);

    yawIntegral_ += error * dt;
    yawIntegral_ = constrain(yawIntegral_, -5.0f, 5.0f);

    float deriv = (error - yawPrevErr_) / dt;
    yawPrevErr_ = error;

    float output = kYawKp * error + kYawKi * yawIntegral_ + kYawKd * deriv;
    return constrain(output, -kYawMax, kYawMax);
}

void OdomMovement::isrUL_A()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->UL_.last_pulse_us;
    instance_->UL_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->UL_, p);
}

void OdomMovement::isrUL_B()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->UL_.last_pulse_us;
    instance_->UL_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->UL_, p);
}

void OdomMovement::isrUR_A()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->UR_.last_pulse_us;
    instance_->UR_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->UR_, p);
}

void OdomMovement::isrUR_B()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->UR_.last_pulse_us;
    instance_->UR_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->UR_, p);
}

void OdomMovement::isrLR_A()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->LR_.last_pulse_us;
    instance_->LR_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->LR_, p);
}

void OdomMovement::isrLR_B()
{
    if (!instance_) return;
    unsigned long n = micros();
    unsigned long p = n - instance_->LR_.last_pulse_us;
    instance_->LR_.last_pulse_us = n;
    if (p > 200) pushPeriod(instance_->LR_, p);
}

void OdomMovement::isrLL()
{
    if (!instance_) return;
    if (digitalRead(instance_->encLL_B_) == HIGH)
        instance_->ticksLL_count_++;
    else
        instance_->ticksLL_count_--;
}

float OdomMovement::measureRPM_LL(float dtSec)
{
    noInterrupts();
    long cur = ticksLL_count_;
    interrupts();

    long dTicks = cur - prevTicksLL_;
    prevTicksLL_ = cur;

    float mag = ((float)abs(dTicks) / 495.0f) / dtSec * 60.0f;
    return (LL_.setpoint >= 0.0f) ? mag : -mag;
}

float OdomMovement::measureRPM(Motor& m)
{
    noInterrupts();
    unsigned long buf[FILTER_SIZE];
    for (uint8_t i = 0; i < FILTER_SIZE; i++) buf[i] = m.period_buf[i];
    unsigned long last = m.last_pulse_us;
    bool has = m.got_pulse;
    interrupts();

    if (!has) return 0.0f;
    if (micros() - last > 200000UL) return 0.0f;

    unsigned long sum = 0;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < FILTER_SIZE; i++)
    {
        if (buf[i] > 0) { sum += buf[i]; cnt++; }
    }
    if (cnt == 0) return 0.0f;

    float avg = (float)(sum / cnt);
    float mag = 60000000.0f / (avg * 4.0f * m.PPR);
    return (m.setpoint >= 0.0f) ? mag : -mag;
}

void OdomMovement::setMotorPWM(Motor& m, float pwm)
{
    if (pwm == 0.0f)
    {
        analogWrite(m.pwmPin, 0);
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        return;
    }

    float absPwm = fabsf(pwm);
    absPwm = kPwmDeadband + (absPwm / 255.0f) * (kPwmMax - kPwmDeadband);
    absPwm = constrain(absPwm, kPwmDeadband, kPwmMax);

    bool goForward = (pwm > 0.0f);
    if (m.inverted) goForward = !goForward;

    if (goForward)
    {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
    }
    else
    {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
    }

    analogWrite(m.pwmPin, (int)absPwm);
}

void OdomMovement::stopMotor(Motor& m)
{
    analogWrite(m.pwmPin, 0);
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    m.integral = 0.0f;
    m.last_error = 0.0f;
    m.got_pulse = false;
}

void OdomMovement::stopAll()
{
    stopMotor(UL_);
    stopMotor(UR_);
    stopMotor(LL_);
    stopMotor(LR_);
    yawIntegral_ = 0.0f;
    yawPrevErr_ = 0.0f;
}

void OdomMovement::pidStepWithRPM(Motor& m, float rpm, float extraRPM)
{
    float sp = m.setpoint + extraRPM;

    if (sp == 0.0f && m.setpoint == 0.0f)
    {
        stopMotor(m);
        return;
    }

    float error = sp - rpm;
    m.integral += error * kTs;
    m.integral = constrain(m.integral, -150.0f, 150.0f);

    float deriv = (error - m.last_error) / kTs;
    float output = m.Kp * error + m.Ki * m.integral + m.Kd * deriv;
    m.last_error = error;

    if (sp < 0.0f) output = -fabsf(output);
    else           output =  fabsf(output);

    output = constrain(output, -255.0f, 255.0f);
    setMotorPWM(m, output);
}

void OdomMovement::ekfStep(float dt, float rpmUL, float rpmUR, float rpmLL, float rpmLR)
{
    float w_UL = rpmUL * 2 * PI / 60.0f;
    float w_UR = rpmUR * 2 * PI / 60.0f;
    float w_LL = rpmLL * 2 * PI / 60.0f;
    float w_LR = rpmLR * 2 * PI / 60.0f;

    const float k = kInvSqrt2 * kWheelRadius / 4.0f;

    float vx = ( w_UL + w_UR + w_LL + w_LR) * k * kOdomScale;
    float vy = ( w_UL - w_UR - w_LL + w_LR) * k * kOdomScale;

    bno_.update();
    float z_th = bno_.getYaw();

    float c = cosf(ekf_th_), s = sinf(ekf_th_);
    float xp = ekf_x_ + (vx * c - vy * s) * dt;
    float yp = ekf_y_ + (vx * s + vy * c) * dt;
    float tp = ekf_th_;

    float F[3][3] = {{1,0,(-vx*s-vy*c)*dt},{0,1,(vx*c-vy*s)*dt},{0,0,1}};
    float FP[3][3] = {0}, Pp[3][3] = {0};

    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k2=0;k2<3;k2++) FP[i][j] += F[i][k2] * P_[k2][j];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k2=0;k2<3;k2++) Pp[i][j] += FP[i][k2] * F[j][k2];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Pp[i][j] += Q_[i][j];

    float yt = wrapPi(z_th - tp);
    float Sc = Pp[2][2] + R_theta_;
    float K[3] = {Pp[0][2]/Sc, Pp[1][2]/Sc, Pp[2][2]/Sc};

    ekf_x_ = xp + K[0] * yt;
    ekf_y_ = yp + K[1] * yt;
    ekf_th_ = wrapPi(tp + K[2] * yt);

    float Pn[3][3];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Pn[i][j] = Pp[i][j];
    for (int i=0;i<3;i++)
    {
        Pn[i][0] -= K[i] * Pp[2][0];
        Pn[i][1] -= K[i] * Pp[2][1];
        Pn[i][2] -= K[i] * Pp[2][2];
    }
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) P_[i][j] = Pn[i][j];
}

void OdomMovement::setRPMs(float ul, float ur, float ll, float lr)
{
    const bool ulChanged = fabsf(UL_.setpoint - ul) > 0.01f;
    const bool urChanged = fabsf(UR_.setpoint - ur) > 0.01f;
    const bool llChanged = fabsf(LL_.setpoint - ll) > 0.01f;
    const bool lrChanged = fabsf(LR_.setpoint - lr) > 0.01f;

    UL_.setpoint = ul;
    UR_.setpoint = ur;
    LL_.setpoint = ll;
    LR_.setpoint = lr;

    // Preserve controller history while the same command is refreshed by the
    // strategy loop. Reset only the wheel whose setpoint actually changed.
    if (ulChanged) { UL_.integral = 0.0f; UL_.last_error = 0.0f; }
    if (urChanged) { UR_.integral = 0.0f; UR_.last_error = 0.0f; }
    if (llChanged) { LL_.integral = 0.0f; LL_.last_error = 0.0f; }
    if (lrChanged) { LR_.integral = 0.0f; LR_.last_error = 0.0f; }

    markCommandReceived();
}

void OdomMovement::begin()
{
    instance_ = this;

    Wire.begin();
    bno_.begin();
    delay(300);

    uint8_t encPins[] = {encUL_A_,encUL_B_,encUR_A_,encUR_B_,encLL_A_,encLL_B_,encLR_A_,encLR_B_};
    for (uint8_t p : encPins) pinMode(p, INPUT_PULLUP);

    uint8_t motPins[] = {pwmUL_,inUL1_,inUL2_,pwmUR_,inUR1_,inUR2_,pwmLL_,inLL1_,inLL2_,pwmLR_,inLR1_,inLR2_};
    for (uint8_t p : motPins) pinMode(p, OUTPUT);

    analogWriteResolution(8);
    analogWriteFrequency(pwmUL_, 25000);
    analogWriteFrequency(pwmUR_, 25000);
    analogWriteFrequency(pwmLL_, 4000);
    analogWriteFrequency(pwmLR_, 25000);

    attachInterrupt(digitalPinToInterrupt(encUL_A_), isrUL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUL_B_), isrUL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUR_A_), isrUR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encUR_B_), isrUR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLL_A_), isrLL,   RISING);
    attachInterrupt(digitalPinToInterrupt(encLR_A_), isrLR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLR_B_), isrLR_B, CHANGE);

    bno_.update();
    ekf_th_ = bno_.getYaw();
    yawTarget_ = ekf_th_;
    yawNow_ = ekf_th_;

    lastCycleMs_ = millis();
    lastCommandMs_ = lastCycleMs_;
    commandEnabled_ = false;
}

void OdomMovement::setCommandTimeout(uint32_t timeoutMs)
{
    commandTimeoutMs_ = timeoutMs;
}

void OdomMovement::markCommandReceived()
{
    lastCommandMs_ = millis();
    commandEnabled_ = true;
}

void OdomMovement::captureCurrentYawTarget()
{
    bno_.update();
    yawTarget_ = bno_.getYaw();
    yawIntegral_ = 0.0f;
    yawPrevErr_ = 0.0f;
}

void OdomMovement::forward(float rpm)
{
    setRPMs(+rpm, +rpm, +rpm, +rpm);
}

void OdomMovement::backward(float rpm)
{
    setRPMs(-rpm, -rpm, -rpm, -rpm);
}

void OdomMovement::right(float rpm)
{
    setRPMs(+rpm, -rpm, -rpm, +rpm);
}

void OdomMovement::left(float rpm)
{
    setRPMs(-rpm, +rpm, +rpm, -rpm);
}

void OdomMovement::stop()
{
    commandEnabled_ = false;
    stopAll();
}

void OdomMovement::resetPose()
{
    ekf_x_ = 0.0f;
    ekf_y_ = 0.0f;
}

void OdomMovement::update()
{
    uint32_t now = millis();

    // A future DriveControlTask must fail safe if RobotTask stops publishing.
    if (commandEnabled_ && (now - lastCommandMs_) > commandTimeoutMs_)
    {
        commandEnabled_ = false;
        stopAll();
    }

    if (now - lastCycleMs_ < (uint32_t)(kTs * 1000)) return;

    float dt = (now - lastCycleMs_) / 1000.0f;
    lastCycleMs_ = now;

    float rpmUL = measureRPM(UL_);
    float rpmUR = measureRPM(UR_);
    float rpmLL = measureRPM_LL(dt);
    float rpmLR = measureRPM(LR_);

    // ← AGREGAR ESTAS 4 LÍNEAS
    lastRpmUL_ = rpmUL;
    lastRpmUR_ = rpmUR;
    lastRpmLL_ = rpmLL;
    lastRpmLR_ = rpmLR;

    bno_.update();
    yawNow_ = bno_.getYaw();
    float omega = yawPidStep(yawNow_, dt);

    float omegaUL = +omega;
    float omegaUR = -omega;
    float omegaLL = +omega;
    float omegaLR = -omega;

    ekfStep(dt, rpmUL, rpmUR, rpmLL, rpmLR);

    pidStepWithRPM(UL_, rpmUL, omegaUL);
    pidStepWithRPM(UR_, rpmUR, omegaUR);
    pidStepWithRPM(LL_, rpmLL, omegaLL);
    pidStepWithRPM(LR_, rpmLR, omegaLR);
}

float OdomMovement::getX() const
{
    return ekf_x_;
}

float OdomMovement::getY() const
{
    return ekf_y_;
}

float OdomMovement::getTheta() const
{
    return ekf_th_;
}

float OdomMovement::getThetaDeg() const
{
    return ekf_th_ * 180.0f / PI;
}

float OdomMovement::getDistance() const
{
    return sqrtf(ekf_x_ * ekf_x_ + ekf_y_ * ekf_y_);
}

float OdomMovement::getForwardProgress() const
{
    return fabs(ekf_x_);
}

float OdomMovement::getLateralProgress() const
{
    return fabs(ekf_y_);
}

float OdomMovement::getYawNow() const
{
    return yawNow_;
}

// qtr correction
void OdomMovement::setTranslation(float vx_rpm, float vy_rpm)
{
    // Cinemática inversa mecanum:
    // UL = +vy +vx   (diagonal izquierda superior)
    // UR = +vy -vx   (diagonal derecha superior)
    // LL = +vy -vx   (diagonal izquierda inferior)
    // LR = +vy +vx   (diagonal derecha inferior)
    float ul = +vy_rpm + vx_rpm;
    float ur = +vy_rpm - vx_rpm;
    float ll = +vy_rpm - vx_rpm;
    float lr = +vy_rpm + vx_rpm;
    setRPMs(ul, ur, ll, lr);
}
