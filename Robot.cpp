#include <Arduino.h>
#include "Robot.h"

Robot::Robot() : _armServo(35, 140), _camServo(47, 137), _imu(43), _led(39), _rgb(36, 35, 37)
{
    //Initialize motors on heap
    const size_t motorNum = 2;
    unsigned int motorPorts[motorNum] = {Motor_Wrapper::SHIELD_M1, Motor_Wrapper::SHIELD_M3};
    _motors = new Motor_Wrapper(motorPorts, motorNum);

    //Initialize ultrasonics on heap
    const size_t ultrasonicsNum = 4;
    unsigned int echoPins[ultrasonicsNum] = {53, 51, 49, 47};
    const unsigned int trigPin = 52;
    const unsigned int burstFrequency = 20;
    _ultrasonics = new Ultrasonic_Wrapper(trigPin, echoPins, burstFrequency, ultrasonicsNum);
}

void Robot::begin(void (*ultrasonicISRs[])())
{
    //Configure motors
    unsigned int encoderPins[_motors->getMotorNum() * Encoder_Wrapper::PINS_PER_SENSOR]
        = {46, 44, 50, 48};
    _motors->setEncoders(encoderPins);
    _motors->setPid(3.5, 10, 0, Motor_Wrapper::MOTOR_LEFT);
    _motors->setPid(3.5, 10, 0, Motor_Wrapper::MOTOR_LEFT);
    _motors->setPid(1, 2.5, 0.15, Motor_Wrapper::MOTOR_DIF);
    _motors->setSpeedMultiplier(Motor_Wrapper::MOTOR_FLIP, Motor_Wrapper::MOTOR_LEFT);
    _motors->setSpeedMultiplier(Motor_Wrapper::MOTOR_NO_FLIP, Motor_Wrapper::MOTOR_RIGHT);
    _motors->begin();

    //Configure servos
    _armServo.attach(Servo_Wrapper::SERVO_S1, 510, 2520);
    _camServo.attach(Servo_Wrapper::SERVO_S2);

    //Configure imu
    adafruit_bno055_offsets_t offsets{13, -48, -25, -101, 127, 438, -2, -2, 0, 1000, 835};
    _imu.setOffsets(offsets);
    _imu.begin();

    //Configre ultrasonics
    _ultrasonics->begin(ultrasonicISRs);

    //Configure led
    _led.begin();

    //Configure rgb led
    _rgb.begin();
}

void Robot::update()
{
    //Update peripherals
    _motors->update();

    _imu.update();

    _ultrasonics->update();

    //Check if robot is turning
    if (isTurning())
    {
        //Check if turn angle has been achieved
        if (fabs(_imu.getYaw() - _startYaw) >= fabs(_turnAngle))
        {
            //Stop motors
            _motors->stop();

            //Indicate that turn is complete
            _isTurning = false;
        }
    }
}

void Robot::run(double leftSpeed, double rightSpeed)
{
    //Run individual motors
    _motors->run(leftSpeed, Motor_Wrapper::MOTOR_LEFT);
    _motors->run(rightSpeed, Motor_Wrapper::MOTOR_RIGHT);

    //Indicate terminated turn in case robot was mid-turn
    _isTurning = false;
}

void Robot::turn(double angle)
{
    //Check if angle is 0
    if (!Utilities::isEqual_DBL(angle, 0))
    {
        //Check if robot is not already turning
        if (!isTurning())
        {
            //Save desired turn angle
            _turnAngle = angle;

            //Save starting angle
            _startYaw = _imu.getYaw();

            //Check if the angle is positive
            if (angle > 0)
            {
                //Run left motor forward
                //Run right motor backward
                run(0.5, -0.5);
            }
            //Angle is negative
            else
            {
                //Run left motor backward
                //Run right motor forward
                run(-0.5, 0.5);
            }

            //Indicate turn in progress
            _isTurning = true;
        }
    }
}

bool Robot::isTurning()
{
    //Return turn status
    return _isTurning;
}

void Robot::captureBall()
{
    //Bring arm to ground
    _armServo.write(_GROUND_ANGLE);
}

void Robot::holdBalls()
{
    //Raise arm to holding angle
    _armServo.write(_HOLD_ANGLE);
}

void Robot::dropBalls()
{
    //Lower arm to dropping angle
    _armServo.write(_DROP_ANGLE);
}

void Robot::setEvac()
{
    //Move arm out of the way
    holdBalls();

    //Set camera for evacuation room
    _camServo.write(_EVAC_ANGLE);
}

void Robot::setLine()
{
    //Move arm out of the way
    holdBalls();

    //Set camera for line following
    _camServo.write(_LINE_ANGLE);
}

bool Robot::nearObstacle()
{
    //Return if robot is less than 100mm away from an obstacle
    return _ultrasonics->getDistance(Ultrasonic_Wrapper::ULTRASONIC_FRONT) <= 100;
}

Motor_Wrapper &Robot::getMotors()
{
    //Return reference to motors
    return *_motors;
}

Servo_Wrapper &Robot::getArmServo()
{
    //Return reference to arm servo
    return _armServo;
}

Servo_Wrapper &Robot::getCamServo()
{
    //Return reference to camera servo
    return _camServo;
}

IMU_Wrapper &Robot::getIMU()
{
    //Return refernce to imu
    return _imu;
}

Ultrasonic_Wrapper &Robot::getUltrasonics()
{
    //Return reference to ultrasonics
    return *_ultrasonics;
}

LED &Robot::getLED()
{
    //Return reference to led
    return _led;
}

RGB_LED &Robot::getRGB_LED()
{
    //Return reference to rgb led
    return _rgb;
}