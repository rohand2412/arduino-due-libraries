#include <Arduino.h>
#include "Motor_Wrapper.h"
#include "Utilities.h"

Motor_Wrapper::Motor_Wrapper(unsigned int* ports,
                             size_t motorNum,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4560*/)
                             : _motorNum(motorNum),
                             _pidNum(_motorNum + 1),
                             _INTERVAL_MS(INTERVAL_MS),
                             _COUNTS_PER_REVOLUTION(COUNTS_PER_REVOLUTION),
                             _RPS_TO_COUNTS_PER_INTERVAL_MS((_INTERVAL_MS * _COUNTS_PER_REVOLUTION) / 1000.0),
                             _COUNTS_PER_INTERVAL_MS_TO_RPS(1000.0 / (_INTERVAL_MS * _COUNTS_PER_REVOLUTION))
{
    _motorsPtr = new Adafruit_DCMotor *[_motorNum];
    _proportionalCoefficients = new double[_pidNum];
    _integralCoefficients = new double[_pidNum];
    _derivativeCoefficients = new double[_pidNum];
    _integrals = new double[_pidNum];
    _lastErrors = new double[_pidNum];
    _speedMultipliers = new int[_motorNum];
    _targetSpeeds_RPS = new double[_motorNum];
    _actualSpeeds_RPS = new double[_motorNum];
    _lastInputtedSpeeds_PWM = new int[_motorNum];
    _updateCounts = new long int[_motorNum];
    _states = new bool[_motorNum];
    _lastCorrected_MS = new unsigned int[_motorNum];
    _elapsedCorrectedTime_MS = new unsigned int[_motorNum];

    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        _motorsPtr[motor] = _motorShield.getMotor(ports[motor]);
        _speedMultipliers[motor] = MOTOR_NO_FLIP;
        _targetSpeeds_RPS[motor] = 0;
        _actualSpeeds_RPS[motor] = 0;
        _lastInputtedSpeeds_PWM[motor] = 0;
        _updateCounts[motor] = 0;
        _states[motor] = false;
        _lastCorrected_MS[motor] = 0;
        _elapsedCorrectedTime_MS[motor] = 0;
    }

    for (size_t pid = 0; pid < _pidNum; pid++)
    {
        _proportionalCoefficients[pid] = 1;
        _integralCoefficients[pid] = 0;
        _derivativeCoefficients[pid] = 0;
        _integrals[pid] = 0;
        _lastErrors[pid] = 0;
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
    delete[] _updateCounts;
    delete[] _states;
    delete[] _lastCorrected_MS;
    delete[] _elapsedCorrectedTime_MS;
}

void Motor_Wrapper::setEncoders(unsigned int* pins)
{
    _encoders.begin(pins, _motorNum);
}

void Motor_Wrapper::setPid(double proportionalCoefficient,
                           double integralCoefficient, 
                           double derivativeCoefficient,
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

void Motor_Wrapper::setPid(double* proportionalCoefficients,
                           double* integralCoefficients,
                           double* derivativeCoefficients)
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
        double newSpeeds[_motorNum];
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            if (getSpeed(motor) != 0 && getState(motor))
            {
                newSpeeds[motor] = _getNewSpeed(motor);
            }
        }

        double correction = _getNewSpeed(MOTOR_DIF);
        int sign = 1;
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            if (getSpeed(motor) != 0 && getState(motor))
            {
                _lastInputtedSpeeds_PWM[motor] = round(newSpeeds[motor] + sign * correction);
                _updateMotor(_getLastInputtedSpeed(motor), motor);
                sign *= -1;
            }
        }

        _justUpdated = true;
        _lastUpdated_MS = millis();
    }
}

bool Motor_Wrapper::getJustUpdated()
{
    if (_justUpdated)
    {
        _justUpdated = false;
        return true;
    }
    else
    {
        return false;
    }
}

unsigned int Motor_Wrapper::getLastCorrected_MS(size_t motor /*= MOTOR_LEFT*/) const
{
    return _lastCorrected_MS[motor];
}

unsigned int Motor_Wrapper::getElapsedCorrectedTime_MS(size_t motor /*= MOTOR_LEFT*/) const
{
    return _elapsedCorrectedTime_MS[motor];
}

long int Motor_Wrapper::getUpdateCounts(size_t motor /*= MOTOR_LEFT*/) const
{
    return _updateCounts[motor];
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

void Motor_Wrapper::setSpeed(double speed, size_t motor /*= MOTOR_ALL*/)
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

void Motor_Wrapper::setSpeed(double* speeds)
{
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        setSpeed(speeds[motor], motor);
    }
}

double Motor_Wrapper::getSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    return _targetSpeeds_RPS[motor];
}

double Motor_Wrapper::getActualSpeed(size_t motor /*= MOTOR_LEFT*/) const
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

void Motor_Wrapper::run(double speed, size_t motor /*= MOTOR_ALL*/)
{
    start();
    setSpeed(speed, motor);
}

void Motor_Wrapper::run(double* speeds)
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
        double individualSpeed = newSpeed * getSpeedMultiplier(motor);
        double inputSpeed = constrain(abs(newSpeed), 0, 255);
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

double Motor_Wrapper::_getNewSpeed(size_t motor /*= MOTOR_LEFT*/)
{
    double error;
    if (motor == MOTOR_DIF)
    {
        error = _lastErrors[MOTOR_LEFT] - _lastErrors[MOTOR_RIGHT];
    }
    else
    {
        double target = getSpeed(motor) * _RPS_TO_COUNTS_PER_INTERVAL_MS;
        _elapsedCorrectedTime_MS[motor] = millis() - _lastCorrected_MS[motor];
        _updateCounts[motor] = getCount(motor);
        double value = _updateCounts[motor] * (double) _INTERVAL_MS / (double) _elapsedCorrectedTime_MS[motor];
        _actualSpeeds_RPS[motor] = value * _COUNTS_PER_INTERVAL_MS_TO_RPS;
        error = target - value;
    }

    _integrals[motor] += error;
    double derivative = error - _lastErrors[motor];

    double correction = Utilities::calculatePid(error, derivative, _integrals[motor],
                                                _proportionalCoefficients[motor],
                                                _integralCoefficients[motor],
                                                _derivativeCoefficients[motor]);

    _lastErrors[motor] = error;

    if (!(motor == MOTOR_DIF))
    {
        _encoders.resetCount(motor);
        _lastCorrected_MS[motor] = millis();
    }

    return correction;
}