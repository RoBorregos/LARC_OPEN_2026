/*
 * @file motors.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Omnidirectional Motor class
 * @version 0.1
 * @date 2026-01-12
 */

#include "motors.hpp"
#include <math.h>

DCMotor::DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2, float d)
        : diameter(d)
{

    in1_pin = in1;
    in2_pin = in2;
    pwm_pin = pwm; 

    encoder_pin1_ = encoder_pin1; 
    encoder_pin2_ = encoder_pin2;

    inverted = invert;

}


DCMotor::DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2)
{

    in1_pin = in1;
    in2_pin = in2;
    pwm_pin = pwm;
    encoder_pin1_ = encoder_pin1;
    encoder_pin2_ = encoder_pin2;

    inverted = invert;

}

DCMotor::~DCMotor()
{
    if (encoder_ != nullptr)
    {
        delete encoder_;
        encoder_ = nullptr;
    }

}



void DCMotor::begin()
{
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);

    pinMode(encoder_pin1_, INPUT_PULLUP);
    pinMode(encoder_pin2_, INPUT_PULLUP);

    encoder_ = new Encoder(encoder_pin1_, encoder_pin2_);
}


void DCMotor::move(int speed, Direction direction)
{
    if (speed < 0) speed = 0;
    if (speed > 255) speed = 255;

    if (inverted)
    {
        direction = (direction == Direction::FORWARD) ? Direction::BACKWARD : Direction::FORWARD;
    }

    if (direction == Direction::FORWARD)
    {
        digitalWrite(in1_pin, HIGH);
        digitalWrite(in2_pin, LOW);
    }
    
    else // BACKWARD
    {
        digitalWrite(in1_pin, LOW);
        
        digitalWrite(in2_pin, HIGH);
    }

    // PWM
    analogWrite(pwm_pin, speed);

    current = direction;
}

void DCMotor::move(int speed)
{

        if (speed > 0)
    {
        move(speed, Direction::FORWARD);
    }
    else if (speed < 0)
    {
        move(abs(speed), Direction::BACKWARD);
    }
    else
    {
        stop();
    }
}


void DCMotor::stop()
{
    analogWrite(pwm_pin, 0);
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
}

void DCMotor::brakeStop()
{
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, HIGH);
    analogWrite(pwm_pin, 0);
}

int DCMotor::getEncoderCount()
{
    return (encoder_ != nullptr) ? (int)encoder_->read() : 0;
}

void DCMotor::resetEncoder() {
    if (encoder_ != nullptr)
        encoder_->write(0);
}

float DCMotor::getDistanceMeters() const {
    if (encoder_ == nullptr) return 0.0f;
    return (float)(encoder_->read()) * diameter * M_PI / ppr_;
}

void DCMotor::setPPR(float ppr) {
    ppr_ = ppr;
}

double DCMotor::getPRotation() {
    return (encoder_ != nullptr) ? (double)encoder_->read() / ppr_ : 0.0;
}


float DCMotor::getPMeters() {
    return (encoder_ != nullptr) ? (float)(encoder_->read() * diameter * M_PI / ppr_) : 0.0f;
}


void DCMotor::testForwardBackward(const char* label){
    const int pwm = 120;
    const uint32_t time = 3000;
    const uint32_t pause = 600;

  Serial.print("===>>> Test DCmotors: ");
  Serial.print(label);
  Serial.println(" <<<===");

  // Forward
  move(pwm);
  delay(time);
  stop(); delay(pause);

    // Backward
  move(-pwm);
  delay(time);
  stop(); delay(pause);

}

