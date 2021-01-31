#include <Arduino.h>
#include "Robot.h"

Robot::Robot() : _armServo(35, 140), _camServo(47, 137), _imu(43)
{
    const size_t motorNum = 2;
    unsigned int motorPorts[motorNum] = {Motor_Wrapper::SHIELD_M1, Motor_Wrapper::SHIELD_M3};
    _motors = new Motor_Wrapper(motorPorts, motorNum);

    const size_t ultrasonicsNum = 4;
    unsigned int echoPins[ultrasonicsNum] = {53, 51, 49, 47};
    const unsigned int trigPin = 52;
    const unsigned int burstFrequency = 20;
    _ultrasonics = new Ultrasonic_Wrapper(trigPin, echoPins, burstFrequency, ultrasonicsNum);
}

void Robot::begin(void (*ultrasonicISRs[])())
{
    unsigned int encoderPins[_motors->getMotorNum() * Encoder_Wrapper::PINS_PER_SENSOR]
        = {46, 44, 50, 48};
    _motors->setEncoders(encoderPins);
    _motors->setPid(3.5, 10, 0, Motor_Wrapper::MOTOR_LEFT);
    _motors->setPid(3.5, 10, 0, Motor_Wrapper::MOTOR_LEFT);
    _motors->setPid(1, 2.5, 0.15, Motor_Wrapper::MOTOR_DIF);
    _motors->setSpeedMultiplier(Motor_Wrapper::MOTOR_FLIP, Motor_Wrapper::MOTOR_LEFT);
    _motors->setSpeedMultiplier(Motor_Wrapper::MOTOR_NO_FLIP, Motor_Wrapper::MOTOR_RIGHT);
    _motors->begin();

    _armServo.attach(Servo_Wrapper::SERVO_S1, 510, 2520);
    _camServo.attach(Servo_Wrapper::SERVO_S2);

    adafruit_bno055_offsets_t offsets{13, -48, -25, -101, 127, 438, -2, -2, 0, 1000, 835};
    _imu.setOffsets(offsets);
    _imu.begin();

    _ultrasonics->begin(ultrasonicISRs);
}

void Robot::update()
{
    _motors->update();

    _imu.update();

    _ultrasonics->update();
}

void Robot::run(double leftSpeed, double rightSpeed)
{
    _motors->run(leftSpeed, Motor_Wrapper::MOTOR_LEFT);
    _motors->run(rightSpeed, Motor_Wrapper::MOTOR_RIGHT);
}

void Robot::captureBall()
{
    _armServo.write(_GROUND_ANGLE);
}

void Robot::holdBalls()
{
    _armServo.write(_HOLD_ANGLE);
}

void Robot::dropBalls()
{
    _armServo.write(_DROP_ANGLE);
}

void Robot::setEvac()
{
    holdBalls();
    _camServo.write(_EVAC_ANGLE);
}

void Robot::setLine()
{

    holdBalls();
    _camServo.write(_LINE_ANGLE);
}

bool Robot::nearObstacle()
{
    return _ultrasonics->getDistance(Ultrasonic_Wrapper::ULTRASONIC_FRONT) <= 100;
}

Motor_Wrapper &Robot::getMotors()
{
    return *_motors;
}

Servo_Wrapper &Robot::getArmServo()
{
    return _armServo;
}

Servo_Wrapper &Robot::getCamServo()
{
    return _camServo;
}

IMU_Wrapper &Robot::getIMU()
{
    return _imu;
}

Ultrasonic_Wrapper &Robot::getUltrasonics()
{
    return *_ultrasonics;
}