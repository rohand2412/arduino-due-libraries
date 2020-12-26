#include <Arduino.h>
#include <Motor_Wrapper.h>

Motor_Wrapper::Motor_Wrapper(unsigned int* ports, size_t motorNum) : _motorNum(motorNum)
{
    _motorsPtr = new Adafruit_DCMotor *[_motorNum];
    _proportionals = new float[_motorNum];
    _integrals = new float[_motorNum];
    _derivatives = new float[_motorNum];
    _speedMultipliers = new int[_motorNum];
    _speeds = new int[_motorNum];
    _states = new bool[_motorNum];

    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        _motorsPtr[motor] = _motorShield.getMotor(ports[motor]);
        _proportionals[motor] = 0;
        _integrals[motor] = 0;
        _derivatives[motor] = 0;
        _speedMultipliers[motor] = MOTOR_NO_FLIP;
        _speeds[motor] = 0;
        _states[motor] = false;
    }
}

Motor_Wrapper::Motor_Wrapper(unsigned int port, size_t motorNum /*= 1*/)
             : Motor_Wrapper(&port, motorNum) {}

Motor_Wrapper::~Motor_Wrapper()
{
    delete[] _motorsPtr;
    delete[] _proportionals;
    delete[] _integrals;
    delete[] _derivatives;
    delete[] _speedMultipliers;
    delete[] _speeds;
    delete[] _states;
}

void Motor_Wrapper::setEncoders(unsigned int* pins)
{
    _encoders.begin(pins, _motorNum);
}

void Motor_Wrapper::setPid(float proportional, float integral, float derivative,
                           size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _proportionals[motor] = proportional;
            _integrals[motor] = integral;
            _derivatives[motor] = derivative;
        }
    }
    else
    {
        _proportionals[motor] = proportional;
        _integrals[motor] = integral;
        _derivatives[motor] = derivative;
    }
}

void Motor_Wrapper::setPid(float* proportionals, float* integrals, float* derivatives)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setPid(proportionals[motor], integrals[motor], derivatives[motor], motor);
    }
}

void Motor_Wrapper::begin()
{
    _motorShield.begin();

    //Warm up motors
    run(150);
    stop();
}

void Motor_Wrapper::update() {}

void Motor_Wrapper::setSpeedMultiplier(int speedMultiplier, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _speedMultipliers[motor] = speedMultiplier;
            _updateMotor(motor);
        }
    }
    else
    {
        _speedMultipliers[motor] = speedMultiplier;
        _updateMotor(motor);
    }
}

void Motor_Wrapper::setSpeedMultiplier(int* speedMultipliers)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setSpeedMultiplier(speedMultipliers[motor], motor);
    }
}

int Motor_Wrapper::getSpeedMultiplier(size_t motor /*= MOTOR_LEFT*/) const
{
    return _speedMultipliers[motor];
}

void Motor_Wrapper::setSpeed(int speed, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _speeds[motor] = speed;
            _updateMotor(motor);
        }
    }
    else
    {
        _speeds[motor] = speed;
        _updateMotor(motor);
    }
}

void Motor_Wrapper::setSpeed(int* speeds)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setSpeed(speeds[motor], motor);
    }
}

int Motor_Wrapper::getSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    return _speeds[motor];
}

void Motor_Wrapper::setState(bool state, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _states[motor] = state;
            _updateMotor(motor);
        }
    }
    else
    {
        _states[motor] = state;
        _updateMotor(motor);
    }
}

void Motor_Wrapper::setState(bool* states)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setState(states[motor], motor);
    }
}

void Motor_Wrapper::start()
{
    setState(MOTOR_ON, MOTOR_ALL);
}

void Motor_Wrapper::stop()
{
    setState(MOTOR_OFF, MOTOR_ALL);
}

bool Motor_Wrapper::getState(size_t motor /*= MOTOR_LEFT*/) const
{
    return _states[motor];
}

void Motor_Wrapper::run(int speed, size_t motor /*= MOTOR_ALL*/)
{
    start();
    setSpeed(speed, motor);
}

void Motor_Wrapper::run(int* speeds)
{
    start();
    setSpeed(speeds);
}

size_t Motor_Wrapper::getMotorNum() const
{
    return _motorNum;
}

long int Motor_Wrapper::getCount(size_t motor /*= MOTOR_LEFT*/)
{
    return _encoders.getCount(motor) * _speedMultipliers[motor];
}

unsigned int Motor_Wrapper::getEncoderPin(size_t sensor /*= Encoder_Wrapper::ENCODER_LEFT*/,
                                          size_t index /*= Encoder_Wrapper::ENCODER_OUT_A*/)
                                          const
{
    return _encoders.getPin(sensor, index);
}

void Motor_Wrapper::_updateMotor(size_t motor /*= MOTOR_LEFT*/)
{
    if (getState(motor))
    {
        int direction;
        int individualSpeed = getSpeed(motor) * getSpeedMultiplier(motor);
        int inputSpeed = getSpeed(motor) < 0 ? -getSpeed(motor) : getSpeed(motor);
        if (individualSpeed == 0)
        {
            direction = RELEASE;
        }
        else if (individualSpeed > 0)
        {
            direction = FORWARD;
        }
        else
        {
            direction = BACKWARD;
        }

        _motorsPtr[motor]->setSpeed(inputSpeed);
        _motorsPtr[motor]->run(direction);
    }
    else
    {
        _motorsPtr[motor]->run(RELEASE);
    }
}