#include <Arduino.h>
#include "Motor_Wrapper.h"
#include "Utilities.h"

Motor_Wrapper::Motor_Wrapper(unsigned int* ports,
                             size_t motorNum,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4460*/)
                             : _motorNum(motorNum),
                             //One more for dif PID
                             _pidNum(_motorNum + 1),
                             _INTERVAL_MS(INTERVAL_MS),
                             _COUNTS_PER_REVOLUTION(COUNTS_PER_REVOLUTION),
                             //Calculate conversion factors
                             _RPS_TO_COUNTS_PER_INTERVAL_MS((_INTERVAL_MS * _COUNTS_PER_REVOLUTION) / 1000.0),
                             _COUNTS_PER_INTERVAL_MS_TO_RPS(1000.0 / (_INTERVAL_MS * _COUNTS_PER_REVOLUTION))
{
    //Allocate memory for motor or pid specific data
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
    _lastNewSpeed_MS = new unsigned int[_motorNum];
    _elapsedNewSpeedTime_MS = new unsigned int[_motorNum];

    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize motor specific data
        _motorsPtr[motor] = _motorShield.getMotor(ports[motor]);
        _speedMultipliers[motor] = MOTOR_NO_FLIP;
        _targetSpeeds_RPS[motor] = 0;
        _actualSpeeds_RPS[motor] = 0;
        _lastInputtedSpeeds_PWM[motor] = 0;
        _updateCounts[motor] = 0;
        _states[motor] = false;
        _lastNewSpeed_MS[motor] = 0;
        _elapsedNewSpeedTime_MS[motor] = 0;
    }

    //Iterate through pids
    for (size_t pid = 0; pid < _pidNum; pid++)
    {
        //Initialize pid specific data
        _proportionalCoefficients[pid] = 1;
        _integralCoefficients[pid] = 0;
        _derivativeCoefficients[pid] = 0;
        _integrals[pid] = 0;
        _lastErrors[pid] = 0;
    }

    //Initialize update specific data
    _lastUpdated_MS = 0;
}

Motor_Wrapper::Motor_Wrapper(unsigned int port,
                             size_t motorNum /*= 1*/,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4460*/)
                             //Reuse alternate constructor
                             : Motor_Wrapper(&port,
                                             motorNum,
                                             INTERVAL_MS,
                                             COUNTS_PER_REVOLUTION)
{}

Motor_Wrapper::~Motor_Wrapper()
{
    //Release heap allocated memory
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
    delete[] _lastNewSpeed_MS;
    delete[] _elapsedNewSpeedTime_MS;
}

void Motor_Wrapper::setEncoders(unsigned int* pins)
{
    //Initialize encoders
    _encoders.createSensor(pins, _motorNum);
}

void Motor_Wrapper::setPid(double proportionalCoefficient,
                           double integralCoefficient, 
                           double derivativeCoefficient,
                           size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Initialize pid data for iterated motor
            _setPid(proportionalCoefficient,
                    integralCoefficient,
                    derivativeCoefficient,
                    motor);
        }
    }
    //Pid data is for single motor
    else
    {
        //Initialize pid data for single motor
        _setPid(proportionalCoefficient,
                integralCoefficient,
                derivativeCoefficient,
                motor);
    }
}

void Motor_Wrapper::setPid(double* proportionalCoefficients,
                           double* integralCoefficients,
                           double* derivativeCoefficients)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize pid data for iterated motor
        _setPid(proportionalCoefficients[motor],
                integralCoefficients[motor],
                derivativeCoefficients[motor],
                motor);
    }
}

void Motor_Wrapper::begin()
{
    //Initialize motor shield
    _motorShield.begin();

    //Warm up motors
    run(0.5);
    //Update motors
    update();
    //Reset state to off
    stop();
    //Reset speed to zero
    setSpeed(0, Motor_Wrapper::MOTOR_ALL);
    //Update motors
    update();
    //Reset encoder counts
    resetCount();
}

void Motor_Wrapper::update()
{
    //Check if _INTERVAL_MS has passed
    if (millis() - _lastUpdated_MS >= _INTERVAL_MS)
    {
        //Declare new speed vars
        double newSpeeds[_motorNum];

        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Check if speed is not 0 and state is on
            if (getSpeed(motor) != 0 && getState(motor))
            {
                //Initialize new speed var with pid result
                newSpeeds[motor] = _getNewSpeed(motor);
            }
        }

        //Initialize correction with dif pid result
        double correction = _getNewSpeed(MOTOR_DIF);

        //Initialize var for alternating correction signs
        int sign = 1;

        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Check if speed is not 0 and state is on
            if (getSpeed(motor) != 0 && getState(motor))
            {
                //Evaluate speed to be inputted and round to int
                _lastInputtedSpeeds_PWM[motor] = round(newSpeeds[motor] + sign * correction);

                //Update motor with new speed to be inputted
                _updateMotor(_getLastInputtedSpeed(motor), motor);

                //Alternate correction signs
                sign *= -1;
            }
        }

        //Trigger flag that update has just been called
        _justUpdated = true;

        //Set last updated time to current time
        _lastUpdated_MS = millis();
    }
}

bool Motor_Wrapper::getJustUpdated()
{
    //Check if update was just called
    if (_justUpdated)
    {
        //Toggle update flag
        _justUpdated = false;

        //Motors were just updated
        return true;
    }
    //Update was not just called
    else
    {
        //Motors were not just updated
        return false;
    }
}

unsigned int Motor_Wrapper::getLastNewSpeed_MS(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return time at which new speed for motor was last generated
    return _lastNewSpeed_MS[motor];
}

unsigned int Motor_Wrapper::getElapsedNewSpeedTime_MS(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return how much time passed between latest generation of new speed for motor
    return _elapsedNewSpeedTime_MS[motor];
}

long int Motor_Wrapper::getUpdateCounts(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return number of counts between latest generation of new speed for motor
    return _updateCounts[motor];
}

void Motor_Wrapper::setSpeedMultiplier(int speedMultiplier, size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Initialize multiplier for iterated motor
            _setSpeedMultiplier(speedMultiplier, motor);
        }
    }
    //Data is for single motor
    else
    {
        //Initialize multiplier for single motor
        _setSpeedMultiplier(speedMultiplier, motor);
    }
}

void Motor_Wrapper::setSpeedMultiplier(int* speedMultipliers)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize multiplier for iterated motor
        _setSpeedMultiplier(speedMultipliers[motor], motor);
    }
}

int Motor_Wrapper::getSpeedMultiplier(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return speed multiplier for motor
    return _speedMultipliers[motor];
}

void Motor_Wrapper::setSpeed(double speed, size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Initialize speed for iterated motor
            _setSpeed(speed, motor);
        }
    }
    //Data is for single motor
    else
    {
        //Initialize speed for single motor
        _setSpeed(speed, motor);
    }
}

void Motor_Wrapper::setSpeed(double* speeds)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize speed for iterated motor
        _setSpeed(speeds[motor], motor);
    }
}

double Motor_Wrapper::getSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return target speed or speed that was inputted by user
    return _targetSpeeds_RPS[motor];
}

double Motor_Wrapper::getActualSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return actual speed at which motor is rotating
    return _actualSpeeds_RPS[motor];
}

void Motor_Wrapper::setState(bool state, size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Initialize state for iterated motor
            _setState(state, motor);
        }
    }
    //Data is for single motor
    else
    {
        //Initialize state for single motor
        _setState(state, motor);
    }
}

void Motor_Wrapper::setState(bool* states)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize state for iterated motor
        _setState(states[motor], motor);
    }
}

void Motor_Wrapper::start()
{
    //Turn on all motors
    setState(MOTOR_ON, MOTOR_ALL);
}

void Motor_Wrapper::stop()
{
    //Turn off all motors
    setState(MOTOR_OFF, MOTOR_ALL);
}

bool Motor_Wrapper::getState(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return state of motor
    return _states[motor];
}

void Motor_Wrapper::run(double speed, size_t motor /*= MOTOR_ALL*/)
{
    //Start motor
    setState(MOTOR_ON, motor);
    //Set speed of motor
    setSpeed(speed, motor);
}

void Motor_Wrapper::run(double* speeds)
{
    //Start all motors
    start();
    //Set individual speeds of all motors
    setSpeed(speeds);
}

size_t Motor_Wrapper::getMotorNum() const
{
    //Return number of motors
    return _motorNum;
}

void Motor_Wrapper::resetCount(size_t motor /*= MOTOR_ALL*/)
{
    //Reset encoder counts of motor
    _encoders.resetCount(motor);
}

long int Motor_Wrapper::getCount(size_t motor /*= MOTOR_LEFT*/)
{
    //Return encoder counts with correct sign
    return _encoders.getCount(motor) * _speedMultipliers[motor];
}

long int Motor_Wrapper::getCountsPerRev() const
{
    return _COUNTS_PER_REVOLUTION;
}

unsigned int Motor_Wrapper::getEncoderPin(size_t sensor /*= Encoder_Wrapper::ENCODER_LEFT*/,
                                          size_t index /*= Encoder_Wrapper::ENCODER_OUT_A*/)
                                          const
{
    //Return specific pin of specific encoder
    return _encoders.getPin(sensor, index);
}

int Motor_Wrapper::_getLastInputtedSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return last pwm speed that was inputted to motor
    return _lastInputtedSpeeds_PWM[motor];
}

void Motor_Wrapper::_updateMotor(int newSpeed, size_t motor /*= MOTOR_LEFT*/)
{
    //Check if motor is on
    if (getState(motor))
    {
        //Declare direction var
        int direction;

        //New speed of motor with correct sign
        double individualSpeed = newSpeed * getSpeedMultiplier(motor);

        //Constrained new speed between 0 and 255
        double inputSpeed = constrain(abs(newSpeed), 0, 255);

        //Check if speed is 0 aka off
        if (individualSpeed == 0)
        {
            //Cut power to motors
            direction = RELEASE;
        }
        //Speed of motor is positive
        else if (individualSpeed > 0)
        {
            //Motors will go forward
            direction = FORWARD;
        }
        //Speed of motor is negative
        else
        {
            //Motors will go backward
            direction = BACKWARD;
        }

        //Input speed to motor
        _motorsPtr[motor]->setSpeed(inputSpeed);
        //Set direction of motor
        _motorsPtr[motor]->run(direction);
    }
    //Motor is off
    else
    {
        //Cut power to motors
        _motorsPtr[motor]->run(RELEASE);
    }
}

double Motor_Wrapper::_getNewSpeed(size_t pid /*= MOTOR_LEFT*/)
{
    //Declare error var
    double error;

    //Check if pid is dif pid
    if (pid == MOTOR_DIF)
    {
        //Error is dif of left and right motor errors
        error = _lastErrors[MOTOR_LEFT] - _lastErrors[MOTOR_RIGHT];
    }
    //Pid is a motor pid
    else
    {
        //Convert target to counts per interval ms
        double target = getSpeed(pid) * _RPS_TO_COUNTS_PER_INTERVAL_MS;

        //Calculate elapsed time from last generation of a new speed
        _elapsedNewSpeedTime_MS[pid] = millis() - _lastNewSpeed_MS[pid];

        //Retrieve encoder counts of motor
        _updateCounts[pid] = getCount(pid);

        //Scale counts to interval ms in case of late entry into _getNewSpeed
        double value = _updateCounts[pid] * (double) _INTERVAL_MS / (double) _elapsedNewSpeedTime_MS[pid];

        //Convert scaled counts back to rps for actual speed
        _actualSpeeds_RPS[pid] = value * _COUNTS_PER_INTERVAL_MS_TO_RPS;

        //Calculate error
        error = target - value;
    }

    //Increase integral by error
    _integrals[pid] += error;

    //Calculate derivative
    double derivative = error - _lastErrors[pid];

    //Use pid to generate new speed
    double newSpeed = Utilities::calculatePid(error, derivative, _integrals[pid],
                                              _proportionalCoefficients[pid],
                                              _integralCoefficients[pid],
                                              _derivativeCoefficients[pid]);

    //Set last error to current error
    _lastErrors[pid] = error;

    //Check if pid is not dif pid
    if (!(pid == MOTOR_DIF))
    {
        //Reset encoder counts of motor
        _encoders.resetCount(pid);

        //Set last call time to current time
        _lastNewSpeed_MS[pid] = millis();
    }

    //Return computed new speed
    return newSpeed;
}

void Motor_Wrapper::_setPid(double proportionalCoefficient,
                            double integralCoefficient,
                            double derivativeCoefficient,
                            size_t motor)
{
    //Initialize coefficients for motor
    _proportionalCoefficients[motor] = proportionalCoefficient;
    _integralCoefficients[motor] = integralCoefficient;
    _derivativeCoefficients[motor] = derivativeCoefficient;
}

void Motor_Wrapper::_setSpeedMultiplier(int speedMultiplier, size_t motor)
{
    //Initialize multiplier for motor
    _speedMultipliers[motor] = speedMultiplier;
}

void Motor_Wrapper::_setSpeed(double speed, size_t motor)
{
    //Initialize target speed for motor
    _targetSpeeds_RPS[motor] = speed;

    //Check if speed is 0
    if (!speed)
    {
        //Cut power to motor immediately
        _motorsPtr[motor]->run(RELEASE);
    }
}

void Motor_Wrapper::_setState(bool state, size_t motor)
{
    //Initialize state for motor
    _states[motor] = state;

    //Check if state is off
    if (!state)
    {
        //Cut power to motor immediately
        _motorsPtr[motor]->run(RELEASE);
    }
}