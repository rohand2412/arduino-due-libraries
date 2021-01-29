#pragma once
#include <Arduino.h>
#include "Serial_Wrapper.h"
#include "Motor_Wrapper.h"
#include "Servo_Wrapper.h"
#include "IMU_Wrapper.h"
#include "Ultrasonic_Wrapper.h"

class Robot
{
    private:
        Motor_Wrapper *_motors;

        Servo_Wrapper _armServo(35, 135);
        Servo_Wrapper _camServo(47, 137);

        const uint8_t _EVAC_ANGLE = 15;
        const Uint8_t _LINE_ANGLE = 35;

        IMU_Wrapper _imu(43);

        Ultrasonic_Wrapper *_ultrasonics;
    
    public:
        Robot();

        void begin();

        void update();

        void run();

        void turn();

        void pickUpBall();

        void dropBalls();

        void setEvac();

        void setLine();

        const Motor_Wrapper *getMotors() const;

        const Servo_Wrapper *getArmServo() const;

        const Servo_Wrapper *getCamServo() const;

        const IMU_Wrapper *getIMU() const;

        const Ultrasonic_Wrapper *getUltrasonics() const;
}