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

        const size_t _pidNum;
        double *_proportionalCoefficients;
        double *_integralCoefficients;
        double *_derivativeCoefficients;
        double *_integrals;
        double *_lastErrors;

        int* _speedMultipliers;

        double* _targetSpeeds_RPS;
        double* _actualSpeeds_RPS;
        int* _lastInputtedSpeeds_PWM;
        long int *_updateCounts;

        bool* _states;

        const unsigned int _INTERVAL_MS;
        const long int _COUNTS_PER_REVOLUTION;
        const double _RPS_TO_COUNTS_PER_INTERVAL_MS;
        const double _COUNTS_PER_INTERVAL_MS_TO_RPS;

        unsigned int *_lastNewSpeed_MS;
        unsigned int *_elapsedNewSpeedTime_MS;
        unsigned int _lastUpdated_MS;
        bool _justUpdated = false;

    public:
        static const size_t SHIELD_M1 = 1;
        static const size_t SHIELD_M2 = 2;
        static const size_t SHIELD_M3 = 3;
        static const size_t SHIELD_M4 = 4;

        static const size_t MOTOR_LEFT = 0;
        static const size_t MOTOR_RIGHT = 1;
        static const size_t MOTOR_ALL = 0xFFFFFFFF; //= -1
        static const size_t MOTOR_DIF = 2;

        static const int MOTOR_FLIP = -1;
        static const int MOTOR_NO_FLIP = 1;

        static const bool MOTOR_ON = true;
        static const bool MOTOR_OFF = false;

    public:
        Motor_Wrapper(unsigned int* ports, size_t motorNum,
                      unsigned int INTERVAL_MS = 20,
                      long int COUNTS_PER_REVOLUTION = 4560);

        Motor_Wrapper(unsigned int port, size_t motorNum = 1,
                      unsigned int INTERVAL_MS = 20,
                      long int COUNTS_PER_REVOLUTION = 4560);

        ~Motor_Wrapper();

        void setEncoders(unsigned int* pins);

        void setPid(double proportionalCoefficient,
                    double integralCoefficient,
                    double derivativeCoefficient,
                    size_t motor = MOTOR_ALL);

        void setPid(double* proportionalCoefficients,
                    double* integralCoefficients,
                    double* derivativeCoefficients);

        void begin();

        void update();

        bool getJustUpdated();

        unsigned int getLastNewSpeed_MS(size_t motor = MOTOR_LEFT) const;

        unsigned int getElapsedNewSpeedTime_MS(size_t motor = MOTOR_LEFT) const;

        long int getUpdateCounts(size_t motor = MOTOR_LEFT) const;

        void setSpeedMultiplier(int speedMultiplier, size_t motor = MOTOR_ALL);

        void setSpeedMultiplier(int* speedMultipliers);

        int getSpeedMultiplier(size_t motor = MOTOR_LEFT) const;

        void setSpeed(double speed, size_t motor = MOTOR_ALL);

        void setSpeed(double* speeds);

        double getSpeed(size_t motor = MOTOR_LEFT) const;

        double getActualSpeed(size_t motor = MOTOR_LEFT) const;

        void setState(bool state, size_t motor = MOTOR_ALL);

        void setState(bool* states);

        void start();

        void stop();

        bool getState(size_t motor = MOTOR_LEFT) const;

        void run(double speed, size_t motor = MOTOR_ALL);

        void run(double* speeds);

        size_t getMotorNum() const;

        void resetCount(size_t motor = MOTOR_ALL);

        long int getCount(size_t motor = MOTOR_LEFT);

        unsigned int getEncoderPin(size_t sensor = Encoder_Wrapper::ENCODER_LEFT,
                                   size_t index = Encoder_Wrapper::ENCODER_OUT_A) const;
    
    private:
        int _getLastInputtedSpeed(size_t motor = MOTOR_LEFT) const;

        void _updateMotor(int newSpeed, size_t motor = MOTOR_LEFT);

        double _getNewSpeed(size_t pid = MOTOR_LEFT);

        void _setPid(double proportionalCoefficient,
                     double integralCoefficient,
                     double derivativeCoefficient,
                     size_t motor);
        
        void _setSpeedMultiplier(int speedMultiplier, size_t motor);

        void _setSpeed(double speed, size_t motor);

        void _setState(bool state, size_t motor);
};