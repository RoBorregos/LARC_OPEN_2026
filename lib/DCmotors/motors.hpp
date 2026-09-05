/*
 * @file motors.hpp
 *
 * @author Ximena Patricia García Magdaleno
 * 
 * @brief Header file for the Omnidirectional Motor class, which is the class for the Omnidirectional motor.
 * 
 * @version 0.1
 * 
 * @date 2026-01-12
 * 
 * First layer for control of CONTROL (1)
 * 
 */

#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>
#include <Encoder.h>


class DCMotor
{
    public:

        enum class Direction { 
            FORWARD,
            BACKWARD
        };
        

        DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2, float d);

        
        
        DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2);
        
        ~DCMotor();

        DCMotor(const DCMotor&)            = delete;
        DCMotor& operator=(const DCMotor&) = delete;

        void begin();
        void move(int speed, Direction direction);
        void move(int speed);
        void stop();
        void brakeStop();
        int getEncoderCount();
        double getPRotation(); // Position Rotation
        float getPMeters(); // Position Meters
        void  resetEncoder();
        float getDistanceMeters() const;   
        void  setPPR(float ppr);     // to configure the motor

        void testForwardBackward(const char* label = "motor");


    private:
        int in1_pin;
        int in2_pin;
        int pwm_pin;
        int encoder_pin1_;
        int encoder_pin2_;
        bool inverted;
        float diameter;

        Direction current = Direction::FORWARD;
        
        Encoder* encoder_ = nullptr;
    
        float ppr_ = 473.0f;    // pulses per revolution (PPR) (each motor has its own configuration)

};

#endif 