/**
 * Central class with everything robot related in one place
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
#include "Serial_Wrapper.h"
#include "Motor_Wrapper.h"
#include "Encoder_Wrapper.h"
#include "Servo_Wrapper.h"
#include "IMU_Wrapper.h"
#include "Ultrasonic_Wrapper.h"
#include "LED.h"
#include "RGB_LED.h"
#include "Button_Wrapper.h"
#include "Utilities.h"

class Robot
{
    private:
        Motor_Wrapper *_motors;

        Encoder_Wrapper _encoders;

        Servo_Wrapper _armServo;
        Servo_Wrapper _camServo;

        const uint8_t _GROUND_ANGLE = 5;
        const uint8_t _HOLD_ANGLE = 85;
        const uint8_t _DROP_ANGLE = 140;
        const uint8_t _EVAC_ANGLE = 15;
        const uint8_t _LINE_ANGLE = 35;
        const uint8_t _ARM_PRESET_ANGLE = 90;
        const uint8_t _CAM_PRESET_ANGLE = 10;

        IMU_Wrapper _imu;

        Ultrasonic_Wrapper *_ultrasonics;

        LED _led;

        RGB_LED _rgb;

        Button_Wrapper _button;

        bool _isTurning = false;
        double _startYaw = 0;
        double _turnAngle = 0;

        const double _TIRE_CIRCUMFRENCE = 3.9 * PI;
        bool _isDrivingDistance = false;
        long int _distanceCounts = 0;
        long int _averageCounts = 0;

        enum class _State
        {
            DORMANT,
            RESET
        };

        _State _state;

        const uint8_t _RESET_PIN = 34;
        const int32_t _RESET_MESSAGE = 0xFFFF;
        const size_t _RESET_MESSAGE_LEN = 1;

    public:
        Robot();

        void begin(void (*ultrasonicISRs[])(), void (*buttonPinISR)());

        void update();

        bool isDormant() const;

        void run(double leftSpeed, double rightSpeed);

        void runDistance_CM(double speed, int distance);

        void runDistance_CM(double leftSpeed, double rightSpeed, int distance);

        int distanceDriven();

        bool isDrivingDistance() const;

        void turn(double angle, double speed = 0.5);

        double angleTurned();

        bool isTurning() const;

        bool available() const;

        void stop();

        void captureBall();

        void holdBalls();

        void dropBalls();

        void setEvac();

        void setLine();

        bool nearObstacle();

        Motor_Wrapper &getMotors();

        Encoder_Wrapper &getEncoders();

        Servo_Wrapper &getArmServo();

        Servo_Wrapper &getCamServo();

        IMU_Wrapper &getIMU();

        Ultrasonic_Wrapper &getUltrasonics();

        LED &getLED();

        RGB_LED &getRGB_LED();

        Button_Wrapper &getButton();
};