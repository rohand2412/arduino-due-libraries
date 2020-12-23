#pragma once
#include <Arduino.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "Encoder_Wrapper.h"

class Motor_Wrapper
{
    private:
        const size_t _motorNum;
        Adafruit_MotorShield _motorShield;
        Adafruit_DCMotor **_motors;

        Encoder_Wrapper _encoders;

        float *_proportionals;
        float *_integrals;
        float *_derivatives;

        int* _speedMultipliers;

        int* _speeds;

        bool* _states;
    
    public:
        static const size_t SHIELD_M1 = 1;
        static const size_t SHIELD_M2 = 2;
        static const size_t SHIELD_M3 = 3;
        static const size_t SHIELD_M4 = 4;

        static const size_t MOTOR_LEFT = 0;
        static const size_t MOTOR_RIGHT = 1;
        static const size_t MOTOR_ALL = 0xFFFFFFFF; //= -1

    public:
        Motor_Wrapper(unsigned int* ports, size_t motorNum);

        Motor_Wrapper(unsigned int port, size_t motorNum);

        ~Motor_Wrapper();

        void setEncoders(unsigned int* pins, size_t sensorNum = 1);

        void setPid(float* proportionals, float* integrals, float* derivatives);

        void setPid(float proportional, float integral, float derivative,
                    size_t motor = MOTOR_ALL);

        void update();

        void setSpeedMultiplier(int* speedMultipliers);

        void setSpeedMultiplier(int speedMultiplier, size_t motor = MOTOR_ALL);

        int getSpeedMultiplier(size_t motor = MOTOR_LEFT) const;

        void setSpeed(int* speeds);

        void setSpeed(int speed, size_t motor = MOTOR_ALL);

        int getSpeed(size_t motor = MOTOR_LEFT) const;

        void setState(bool* states);

        void setState(bool state, size_t motor = MOTOR_ALL);

        void start();

        void stop();

        bool getState(size_t motor = MOTOR_LEFT) const;

        void run(int* speeds);

        void run(int speed, size_t motor = MOTOR_ALL);

        size_t getMotorNum() const;
    
    private:
        float _calculatePid(size_t motor = MOTOR_LEFT);
}