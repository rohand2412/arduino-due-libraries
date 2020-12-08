#pragma once
#include <Arduino.h>
#include "IMU.h"

class IMU_Wrapper : public IMU
{
    private:
        float _oldYawRaw;
        float _oldPitchRaw;
        float _oldRollRaw;
        float _yaw;
        float _pitch;
        float _roll;
    
    public:
        IMU_Wrapper(unsigned int mpuInterruptPin);

        void update();

        void reset(float yaw = 0, float pitch = 0, float roll = 0);

        float getYaw() const;

        float getPitch() const;

        float getRoll() const;
    
    private:
        void _overflow(float& oldRaw, float& raw, float& axis);
};