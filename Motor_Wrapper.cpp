#include <Arduino.h>
#include "Motor_Wrapper.h"
#include "Utilities.h"

Motor_Wrapper::Motor_Wrapper(unsigned int* ports,
                             size_t motorNum,
                             unsigned int INTERVAL_MS /*= 20*/,
                             long int COUNTS_PER_REVOLUTION /*= 4460*/)
                             : _motorNum(motorNum),
                             _INTERVAL_MS(INTERVAL_MS),
                             _COUNTS_PER_REVOLUTION(COUNTS_PER_REVOLUTION),
                             //Calculate conversion factors
                             _RPS_TO_COUNTS_PER_INTERVAL_MS((_INTERVAL_MS * _COUNTS_PER_REVOLUTION) / 1000.0),
                             _COUNTS_PER_INTERVAL_MS_TO_RPS(1000.0 / (_INTERVAL_MS * _COUNTS_PER_REVOLUTION))
{
    //Allocate memory for motor or pid specific data
    _motorsPtr = new Adafruit_DCMotor *[_motorNum];
    _PidPtr = new PID *[_motorNum];
    _initializedPid = new bool [_motorNum];
    _mode = new bool[_motorNum];
    _speedMultipliers = new int[_motorNum];
    _inputs = new double [_motorNum];
    _outputs = new double [_motorNum];
    _setpoints = new double [_motorNum];
    _states = new bool[_motorNum];

    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize motor specific data
        _motorsPtr[motor] = _motorShield.getMotor(ports[motor]);
        _speedMultipliers[motor] = MOTOR_NO_FLIP;
        _states[motor] = false;

        //Initialize pid specific data
        _initializedPid[motor] = false;
        _mode[motor] = AUTOMATIC;
        _inputs[motor] = 0.0f;
        _outputs[motor] = 0.0f;
        _setpoints[motor] = 0.0f;
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
    delete[] _initializedPid;
    delete[] _speedMultipliers;
    delete[] _states;
    delete[] _inputs;
    delete[] _outputs;
    delete[] _setpoints;

    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        delete _PidPtr[motor];
    }
    delete[] _PidPtr;
}

void Motor_Wrapper::setEncoders(unsigned int* pins)
{
    //Initialize encoders
    _encoders.createSensor(pins, _motorNum);
}

void Motor_Wrapper::setPid(double kp, double ki, double kd, size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Initialize pid data for iterated motor
            _setPid(kp, ki, kd, motor);
        }
    }
    //Pid data is for single motor
    else
    {
        //Initialize pid data for single motor
        _setPid(kp, ki, kd, motor);
    }
}

void Motor_Wrapper::setPid(double* kps, double* kis, double* kds)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Initialize pid data for iterated motor
        _setPid(kps[motor], kis[motor], kds[motor], motor);
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
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Update PID controller
        _PidPtr[motor]->Compute();

        if (_PidPtr[motor]->GetMode() == AUTOMATIC)
        {
            Serial.print("AUTO");
        }
        else
        {
            Serial.print("MANU");
        }
        Serial.print("\t");
    }

    //Check if _INTERVAL_MS has passed
    if (millis() - _lastUpdated_MS >= _INTERVAL_MS)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Update input to the PID
            _updateInput(millis() - _lastUpdated_MS, motor);
        }

        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Check if speed is not 0 and state is on
            if (getState(motor))
            {
                //Check if Pid is being used and if setpoint is not 0
                //or if Pid is not being used and if output is not 0
                if ((!Utilities::isEqual_DBL(_setpoints[motor], 0) && _mode[motor] == AUTOMATIC)
                    || (!Utilities::isEqual_DBL(_outputs[motor], 0) && _mode[motor] == MANUAL))
                {
                    //Update motor with new speed to be inputted
                    _updateMotor((int)_outputs[motor], motor);
                }
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

double Motor_Wrapper::getOutput(size_t motor /*= MOTOR_LEFT*/) const
{
    return _outputs[motor];
}

double Motor_Wrapper::getInput(size_t motor /*= MOTOR_LEFT*/) const
{
    return _inputs[motor];
}

void Motor_Wrapper::setPwm(int pwm, size_t motor /*= MOTOR_ALL*/)
{
    //Check if data is for all motors
    if (motor == MOTOR_ALL)
    {
        //Iterate through motors
        for (size_t motor = 0; motor < _motorNum; motor++)
        {
            //Set pwm for iterated motor
            _setPwm(pwm, motor);
        }
    }
    //Data is for single motor
    else
    {
        //Set pwm for single motor
        _setPwm(pwm, motor);
    }
}

void Motor_Wrapper::setPwm(int* pwms)
{
    //Iterate through motors
    for (size_t motor = 0; motor < _motorNum; motor++)
    {
        //Set pwm for iterated motor
        _setPwm(pwms[motor], motor);
    }
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
    return _setpoints[motor] * _COUNTS_PER_INTERVAL_MS_TO_RPS;
}

double Motor_Wrapper::getActualSpeed(size_t motor /*= MOTOR_LEFT*/) const
{
    //Return actual speed at which motor is rotating
    return _inputs[motor] * _COUNTS_PER_INTERVAL_MS_TO_RPS;
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

void Motor_Wrapper::_updateInput(unsigned int elapsedTime, size_t motor)
{
    //Check if Pid is enabled
    if (_PidPtr[motor]->GetMode() == AUTOMATIC)
    {
        //Fetch and scale encoder readings
        _inputs[motor] = getCount(motor) * (double)_INTERVAL_MS / (double)elapsedTime;

        //Reset for next reading
        resetCount(motor);
    }
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

void Motor_Wrapper::_setPid(double kp, double ki, double kd, size_t motor)
{
    if (!_initializedPid[motor])
    {
        //Cancel out internal conversion
        ki *= 100 / _INTERVAL_MS;
        kd /= 100 / _INTERVAL_MS;

        //Initialize coefficients for motor
        _PidPtr[motor] = new PID(&_inputs[motor], &_outputs[motor], &_setpoints[motor], kp, ki, kd, DIRECT);

        //Configure PID
        _PidPtr[motor]->SetSampleTime(_INTERVAL_MS);
        _PidPtr[motor]->SetMode(AUTOMATIC);
        _PidPtr[motor]->SetOutputLimits(-255, 255);

        //Indicate that PID is now initialized
        _initializedPid[motor] = true;
    }
    else
    {
        if (!Serial)
        {
            Serial.begin(112500);
        }
        Serial.println("[ERROR] PID HAS ALREADY BEEN INITIALIZED");
    }
}

void Motor_Wrapper::_setPwm(int pwm, size_t motor)
{
    //Set the new output to be the pwm
    _outputs[motor] = pwm;

    //Check if we just came from _setSpeed
    if (_mode[motor] == AUTOMATIC)
    {
        //Disable the Pid
        _PidPtr[motor]->SetMode(MANUAL);

        //Indicate manual mode
        _mode[motor] = MANUAL;
    }

    //Check if pwm is 0
    if (!pwm)
    {
        //Cut power immediately
        _motorsPtr[motor]->run(RELEASE);
    }
    else
    {
        //No need for clause to re-enable Pid
        //Pid is never used in _setPwm
    }
}

void Motor_Wrapper::_setSpeedMultiplier(int speedMultiplier, size_t motor)
{
    //Initialize multiplier for motor
    _speedMultipliers[motor] = speedMultiplier;
}

void Motor_Wrapper::_setSpeed(double speed, size_t motor)
{
    //Initialize target speed for motor
    _setpoints[motor] = speed * _RPS_TO_COUNTS_PER_INTERVAL_MS;

    //Indicate automatic mode
    _mode[motor] = AUTOMATIC;

    //Check if speed is 0
    if (Utilities::isEqual_DBL(speed, 0))
    {
        //Cut power to motor immediately
        _motorsPtr[motor]->run(RELEASE);

        //Disable PID
        _PidPtr[motor]->SetMode(MANUAL);

        //mode stays AUTOMATIC because
        //motor is still using _setpoint[motor]
        //or RPS and not _output[motor] or pwm
    }
    else
    {
        //Check if motor has state of on
        //but Pid was disabled due to 0 speed
        if (_states[motor] && _PidPtr[motor]->GetMode() == MANUAL)
        {
            //Re-enable Pid
            _PidPtr[motor]->SetMode(AUTOMATIC);
        }
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

        //Disable PID
        _PidPtr[motor]->SetMode(MANUAL);

        //mode is unchanged because state
        //does not affect whether the motor
        //is in a using Pid or nonusing Pid
        //state
    }
    else
    {
        //Check if current Pid mode matches motor mode
        //in case state was just off
        if (_PidPtr[motor]->GetMode() != _mode[motor])
        {
            //Check if Pid is being used and if setpoint is not 0
            //or if Pid is not being used and if output is not 0
            if ((!Utilities::isEqual_DBL(_setpoints[motor], 0) && _mode[motor] == AUTOMATIC)
                || (!Utilities::isEqual_DBL(_outputs[motor], 0) && _mode[motor] == MANUAL))
            {
                //Resume Pid to intended mode
                _PidPtr[motor]->SetMode(_mode[motor]);
            }
        }
    }
}