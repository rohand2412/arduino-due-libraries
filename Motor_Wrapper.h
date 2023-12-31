/**
 * Wraps Adafruit's motor drivers with PID and other functionality
 * Copyright (C) 2022  Rohan Dugad
 * 
 * Contact info:
 * https://docs.google.com/document/d/17IhBs4cz7FXphE0praCaWMjz016a7BFU5IQbm1CNnUc/edit?usp=sharing
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <Arduino.h>
#include "PID_v1.h"
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "Encoder_Wrapper.h"

class Motor_Wrapper
{
    private:
        const size_t _motorNum;
        Adafruit_MotorShield _motorShield;
        Adafruit_DCMotor **_motorsPtr;

        PID **_PidPtr;
        bool *_initializedPid;
        bool *_mode;

        Encoder_Wrapper _encoders;

        int* _speedMultipliers;

        double *_inputs;
        double *_outputs;
        double *_setpoints;

        double *_forwardCoefs;
        double *_backwardCoefs;

        bool* _states;

        const unsigned int _INTERVAL_MS;
        const long int _COUNTS_PER_REVOLUTION;
        const double _RPS_TO_COUNTS_PER_INTERVAL_MS;
        const double _COUNTS_PER_INTERVAL_MS_TO_RPS;

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

        static const size_t COEF_NUM = 3;
        static const size_t KP_INDEX = 0;
        static const size_t KI_INDEX = 1;
        static const size_t KD_INDEX = 2;

        static const bool COEF_FORWARD = 1;
        static const bool COEF_BACKWARD = 0;

    public:
        Motor_Wrapper(unsigned int* ports, size_t motorNum,
                      unsigned int INTERVAL_MS = 20,
                      long int COUNTS_PER_REVOLUTION = 4460);

        Motor_Wrapper(unsigned int port, size_t motorNum = 1,
                      unsigned int INTERVAL_MS = 20,
                      long int COUNTS_PER_REVOLUTION = 4460);

        ~Motor_Wrapper();

        void setEncoders(unsigned int* pins);

        void setPid(double forwardKp, double forwardKi, double forwardKd,
                    double backwardKp, double backwardKi, double backwardKd,
                    size_t motor = MOTOR_ALL);

        void setPid(double* forwardKps, double* forwardKis, double* forwardKds,
                    double* backwardKps, double* backwardKis, double* backwardKds);

        double getCoef(size_t coefIndex, bool coefType, size_t motor = MOTOR_LEFT) const;

        double getActualCoef(size_t coefIndex, size_t motor = MOTOR_LEFT);

        void begin();

        void update();

        bool getJustUpdated();

        double getOutput(size_t motor = MOTOR_LEFT) const;

        double getInput(size_t motor = MOTOR_LEFT) const;

        bool getMode(size_t motor = MOTOR_LEFT) const;

        bool getActualMode(size_t motor = MOTOR_LEFT);

        void setPwm(int pwm, size_t motor = MOTOR_ALL);

        void setPwm(int *pwms);

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

        long int getCountsPerRev() const;

        unsigned int getEncoderPin(size_t sensor = Encoder_Wrapper::ENCODER_LEFT,
                                   size_t index = Encoder_Wrapper::ENCODER_OUT_A) const;
    
    private:
        void _updateInput(unsigned int elapsedTime, size_t motor);

        void _updateMotor(int newSpeed, size_t motor = MOTOR_LEFT);

        void _setPid(double forwardKp, double forwardKi, double forwardKd,
                     double backwardKp, double backwardKi, double backwardKd,
                     size_t motor);

        void _setPwm(int pwm, size_t motor);

        void _setSpeedMultiplier(int speedMultiplier, size_t motor);

        void _setSpeed(double speed, size_t motor);

        void _setState(bool state, size_t motor);
};