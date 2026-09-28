#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <PID_v1.h>
#include <Arduino.h> 

class PIDController {

    public:
        PIDController();

        PIDController(float kp, float ki, float kd, float outputMin, float outputMax);

        ~PIDController();

        float update(float measurement, float setpoint);

        void reset();

        void resetToMeasurement(float measurement, float setpoint);

        void setGains(float kp, float ki, float kd);

        void setOutputLimits(float min, float max);

        void setSampleTime(int ms);

        void setEnabled(bool enabled);

        bool isEnabled() const;

        float getError() const;

        float getOutput() const;

        void setAngleWrapping(bool enabled);

        bool isAngleWrappingEnabled() const;

    private:

        double lastMeasurement_;
        double lastSetpoint_;
        double output_;
        bool enabled_;
        bool angleWrapping_;
        PID* pid_;
};

#endif