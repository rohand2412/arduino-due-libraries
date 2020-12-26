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
        Adafruit_DCMotor **_motorsPtr;

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

        static const int MOTOR_FLIP = -1;
        static const int MOTOR_NO_FLIP = 1;

        static const bool MOTOR_ON = true;
        static const bool MOTOR_OFF = false;

    public:
        Motor_Wrapper(unsigned int* ports, size_t motorNum);

        Motor_Wrapper(unsigned int port, size_t motorNum = 1);

        ~Motor_Wrapper();

        void setEncoders(unsigned int* pins);

        void setPid(float proportional, float integral, float derivative,
                    size_t motor = MOTOR_ALL);

        void setPid(float* proportionals, float* integrals, float* derivatives);

        void begin();

        void update();

        void setSpeedMultiplier(int speedMultiplier, size_t motor = MOTOR_ALL);

        void setSpeedMultiplier(int* speedMultipliers);

        int getSpeedMultiplier(size_t motor = MOTOR_LEFT) const;

        void setSpeed(int speed, size_t motor = MOTOR_ALL);

        void setSpeed(int* speeds);

        int getSpeed(size_t motor = MOTOR_LEFT) const;

        void setState(bool state, size_t motor = MOTOR_ALL);

        void setState(bool* states);

        void start();

        void stop();

        bool getState(size_t motor = MOTOR_LEFT) const;

        void run(int speed, size_t motor = MOTOR_ALL);

        void run(int* speeds);

        size_t getMotorNum() const;

        long int getCount(size_t motor = MOTOR_LEFT);

        unsigned int getEncoderPin(size_t sensor = Encoder_Wrapper::ENCODER_LEFT,
                                   size_t index = Encoder_Wrapper::ENCODER_OUT_A) const;
    
    private:
        void _updateMotor(size_t motor = MOTOR_LEFT);

        float _calculatePid(size_t motor = MOTOR_LEFT);
};