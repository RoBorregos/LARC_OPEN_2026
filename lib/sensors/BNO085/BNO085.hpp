/*
 * @file bno085.hpp
 * @author Ximena Patricia García Magdaleno
 * @brief Header file for the BNO085 class.
 * @version 0.1
 * @date 2026-01-12
 */

#ifndef BNO085_HPP
#define BNO085_HPP

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <tuple>
#include "pins.h"

class BNO085
{
public:
    BNO085();

    bool begin();
    void update();

    float getYaw() const;
    float getRoll() const;
    float getPitch() const;

    std::tuple<float, float, float> getLinealAcceleration();

    void getAngular();
    void getAngularPrinted();

private:
    Adafruit_BNO08x bno;

    float yawDeg_   = 0.0f;
    float rollDeg_  = 0.0f;
    float pitchDeg_ = 0.0f;

    float linAccX_ = 0.0f;
    float linAccY_ = 0.0f;
    float linAccZ_ = 0.0f;

    bool initialized;

    float wrapAngle(float angle) const;

    float filteredYawDeg_ = 0.0f;
    bool firstYawSample_  = true;

    uint32_t lastValidMs_ = 0;

    void quaternionToEuler(float qw, float qx, float qy, float qz,
                           float &yaw, float &pitch, float &roll) const;

    void manualReset();
    bool waitForAddress(uint8_t address);
    bool tryAddress(uint8_t address);
    bool enableReports();
    void recover();
};

#endif