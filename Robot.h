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
        int _distanceCounts = 0;

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

        bool isDrivingDistance() const;

        void turn(double angle, double speed = 0.5);

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