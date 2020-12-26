#include <Arduino.h>
#include "Motor_Wrapper.h"
#include "ArduinoTrace.h"

Motor_Wrapper::Motor_Wrapper(unsigned int* ports,
                             size_t motorNum,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4560*/)
                             : _motorNum(motorNum),
                             _INTERVAL_MS(INTERVAL_MS),
                             _COUNTS_PER_REVOLUTION(COUNTS_PER_REVOLUTION),
                             _RPS_TO_COUNTS_PER_INTERVAL_MS((_INTERVAL_MS * _COUNTS_PER_REVOLUTION) / 1000.0),
                             _COUNTS_PER_INTERVAL_MS_TO_RPS(1000.0 / (_INTERVAL_MS * _COUNTS_PER_REVOLUTION))
{
    _motorsPtr = new Adafruit_DCMotor *[_motorNum];
    _proportionalCoefficients = new float[_motorNum];
    _integralCoefficients = new float[_motorNum];
    _derivativeCoefficients = new float[_motorNum];
    _integrals = new float[_motorNum];
    _lastErrors = new float[_motorNum];
    _speedMultipliers = new int[_motorNum];
    _targetSpeeds_RPS = new float[_motorNum];
    _actualSpeeds_RPS = new float[_motorNum];
    _lastInputtedSpeeds_PWM = new int[_motorNum];
    _states = new bool[_motorNum];

    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        _motorsPtr[motor] = _motorShield.getMotor(ports[motor]);
        _proportionalCoefficients[motor] = 1;
        _integralCoefficients[motor] = 0;
        _derivativeCoefficients[motor] = 0;
        _integrals[motor] = 0;
        _lastErrors[motor] = 0;
        _speedMultipliers[motor] = MOTOR_NO_FLIP;
        _targetSpeeds_RPS[motor] = 0;
        _actualSpeeds_RPS[motor] = 0;
        _lastInputtedSpeeds_PWM[motor] = 0;
        _states[motor] = false;
    }

    _lastUpdated_MS = 0;
}

Motor_Wrapper::Motor_Wrapper(unsigned int port,
                             size_t motorNum /*= 1*/,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4560*/)
                             : Motor_Wrapper(&port,
                                             motorNum,
                                             INTERVAL_MS,
                                             COUNTS_PER_REVOLUTION)
{}

Motor_Wrapper::~Motor_Wrapper()
{
    delete[] _motorsPtr;
    delete[] _proportionalCoefficients;
    delete[] _integralCoefficients;
    delete[] _derivativeCoefficients;
    delete[] _integrals;
    delete[] _lastErrors;
    delete[] _speedMultipliers;
    delete[] _targetSpeeds_RPS;
    delete[] _actualSpeeds_RPS;
    delete[] _lastInputtedSpeeds_PWM;
    delete[] _states;
}

void Motor_Wrapper::setEncoders(unsigned int* pins)
{
    _encoders.begin(pins, _motorNum);
}

void Motor_Wrapper::setPid(float proportionalCoefficient,
                           float integralCoefficient, 
                           float derivativeCoefficient,
                           size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _proportionalCoefficients[motor] = proportionalCoefficient;
            _integralCoefficients[motor] = integralCoefficient;
            _derivativeCoefficients[motor] = derivativeCoefficient;
        }
    }
    else
    {
        _proportionalCoefficients[motor] = proportionalCoefficient;
        _integralCoefficients[motor] = integralCoefficient;
        _derivativeCoefficients[motor] = derivativeCoefficient;
    }
}

void Motor_Wrapper::setPid(float* proportionalCoefficients,
                           float* integralCoefficients,
                           float* derivativeCoefficients)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setPid(proportionalCoefficients[motor],
               integralCoefficients[motor],
               derivativeCoefficients[motor],
               motor);
    }
}

void Motor_Wrapper::begin()
{
    _motorShield.begin();

    //Warm up motors
    run(0.5);
    update();
    stop();
    setSpeed(0, Motor_Wrapper::MOTOR_ALL);
    update();
    resetCount();
}

void Motor_Wrapper::update()
{
    if (millis() - _lastUpdated_MS >= _INTERVAL_MS)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            if (getSpeed(motor) != 0 && getState(motor))
            {
                float correction = _getCorrection(motor);
                float newSpeed = correction + _getLastInputtedSpeed(motor);
                _lastInputtedSpeeds_PWM[motor] = newSpeed >= 0 ? newSpeed + 0.5 : newSpeed - 0.5;
                _updateMotor(_getLastInputtedSpeed(motor), motor);
            }
        }
        _encoders.resetCount();
        _lastUpdated_MS = millis();
    }
}

void Motor_Wrapper::setSpeedMultiplier(int speedMultiplier, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _speedMultipliers[motor] = speedMultiplier;
        }
    }
    else
    {
        _speedMultipliers[motor] = speedMultiplier;
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

void Motor_Wrapper::setSpeed(float speed, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _targetSpeeds_RPS[motor] = speed;

            if (!speed)
            {
                _motorsPtr[motor]->run(RELEASE);
            }
        }
    }
    else
    {
        _targetSpeeds_RPS[motor] = speed;

        if (!speed)
        {
            _motorsPtr[motor]->run(RELEASE);
        }
    }
}

void Motor_Wrapper::setSpeed(float* speeds)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setSpeed(speeds[motor], motor);
    }
}

float Motor_Wrapper::getSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    return _targetSpeeds_RPS[motor];
}

float Motor_Wrapper::getActualSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    return _actualSpeeds_RPS[motor];
}

void Motor_Wrapper::setState(bool state, size_t motor /*= MOTOR_ALL*/)
{
    if (motor == MOTOR_ALL)
    {
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            _states[motor] = state;

            if (!state)
            {
                _motorsPtr[motor]->run(RELEASE);
            }
        }
    }
    else
    {
        _states[motor] = state;
        if (!state)
        {
            _motorsPtr[motor]->run(RELEASE);
        }
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

void Motor_Wrapper::run(float speed, size_t motor /*= MOTOR_ALL*/)
{
    start();
    setSpeed(speed, motor);
}

void Motor_Wrapper::run(float* speeds)
{
    start();
    setSpeed(speeds);
}

size_t Motor_Wrapper::getMotorNum() const
{
    return _motorNum;
}

void Motor_Wrapper::resetCount(size_t motor /*= MOTOR_ALL*/)
{
    _encoders.resetCount(motor);
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

int Motor_Wrapper::_getLastInputtedSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    return _lastInputtedSpeeds_PWM[motor];
}

void Motor_Wrapper::_updateMotor(int newSpeed, size_t motor /*= MOTOR_LEFT*/)
{
    if (getState(motor))
    {
        int direction;
        float individualSpeed = newSpeed * getSpeedMultiplier(motor);
        float inputSpeed = newSpeed < 0 ? -newSpeed : newSpeed;
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

float Motor_Wrapper::_getCorrection(size_t motor /*= MOTOR_LEFT*/)
{
    float target = getSpeed(motor) * _RPS_TO_COUNTS_PER_INTERVAL_MS;
    unsigned int elapsedTime = millis() - _lastUpdated_MS;
    long int count = getCount(motor);
    float value = count * (float) _INTERVAL_MS / (float) elapsedTime;
    _actualSpeeds_RPS[motor] = value * _COUNTS_PER_INTERVAL_MS_TO_RPS;

    float error = target - value;
    _integrals[motor] += error;
    float derivative = error - _lastErrors[motor];
    float correction = (error * _proportionalCoefficients[motor])
                       + (_integrals[motor] * _integralCoefficients[motor])
                       + (derivative * _derivativeCoefficients[motor]);
    _lastErrors[motor] = error;

    return correction;
}