#include <Arduino.h>
#include "IMU_Wrapper.h"
#include "Utilities.h"

//Initialize static variables
size_t IMU_Wrapper::_instanceNum = 0;

size_t IMU_Wrapper::_sensorNum = 0;

IMU **IMU_Wrapper::_imusPtr;

unsigned int *IMU_Wrapper::_sensorAddrs;

bool *IMU_Wrapper::_haveBegun;

bool *IMU_Wrapper::_haveSetCrystal;

bool IMU_Wrapper::_createdSensor = false;

IMU_Wrapper::IMU_Wrapper(unsigned int RST, unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/)
{
    //Log initialization of new instance
    _instanceNum++;

    //Create sensor with given params
    createSensor(RST, sensorID, address);
}

IMU_Wrapper::IMU_Wrapper()
{
    //Log initialization of new instance
    _instanceNum++;
}

IMU_Wrapper::~IMU_Wrapper()
{
    //Check if only one instance exists
    if (_instanceNum == 1)
    {
        //Iterate through sensors
        for (size_t sensor = 0; sensor < _sensorNum; sensor++)
        {
            //Destroy each sensor individually
            delete _imusPtr[sensor];
        }

        //Reset sensor number to 0
        _sensorNum = 0;

        //Indicate that sensor has been destroyed
        _createdSensor = false;

        //Delete allocated static variables
        delete[] _imusPtr;
        delete[] _sensorAddrs;
        delete[] _haveBegun;
        delete[] _haveSetCrystal;
    }

    //Indicate removal of instance
    _instanceNum--;
}

void IMU_Wrapper::createSensor(unsigned int RST, unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/)
{
    //Check for single instance
    if (_instanceNum == 1)
    {
        //Check if data is already allocated
        //(coming from second instance perhaps)
        if (!_createdSensor)
        {
            //Indicate new sensor
            _sensorNum++;

            //Allocate memory
            _imusPtr = new IMU *[_sensorNum];
            _sensorAddrs = new unsigned int[_sensorNum];
            _haveBegun = new bool[_sensorNum];
            _haveSetCrystal = new bool[_sensorNum];

            //Iterate through sensors
            for (size_t sensor = 0; sensor < _sensorNum; sensor++)
            {
                //Populate memory
                _imusPtr[sensor] = new IMU(RST, sensorID, address);
                _sensorAddrs[sensor] = address;
                _haveBegun[sensor] = false;
                _haveSetCrystal[sensor] = false;
            }

            //Indicate creation of sensor
            _createdSensor = true;
        }
    }
    //Check if desired sensor is not already created
    else if (Utilities::find(address, _sensorAddrs, _sensorNum) == 0xFFFFFFFF) //== -1
    {
        //Save old data
        size_t oldSensorNum = _sensorNum;
        IMU **imusPtr = _imusPtr;
        unsigned int *sensorAddrs = _sensorAddrs;
        bool *haveBegun = _haveBegun;
        bool *haveSetCrystal = _haveSetCrystal;

        //Indicate new sensor
        _sensorNum++;

        //Allocate new memory
        _imusPtr = new IMU *[_sensorNum];
        _sensorAddrs = new unsigned int[_sensorNum];
        _haveBegun = new bool[_sensorNum];
        _haveSetCrystal = new bool[_sensorNum];

        //Iterate through old sensors
        for (size_t oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
        {
            //Populate new memory with old data
            _imusPtr[oldSensor] = imusPtr[oldSensor];
            _sensorAddrs[oldSensor] = sensorAddrs[oldSensor];
            _haveBegun[oldSensor] = haveBegun[oldSensor];
            _haveSetCrystal[oldSensor] = haveSetCrystal[oldSensor];
        }

        //Iterate through new sensors
        for (size_t newSensor = oldSensorNum; newSensor < _sensorNum; newSensor++)
        {
            //Populate new memory with new data
            _imusPtr[newSensor] = new IMU(RST, sensorID, address);
            _sensorAddrs[newSensor] = address;
            _haveBegun[newSensor] = false;
            _haveSetCrystal[newSensor] = false;
        }

        //Delete old saved data
        delete[] imusPtr;
        delete[] sensorAddrs;
        delete[] haveBegun;
        delete[] haveSetCrystal;
    }

    //Initialize angle arrays
    for (size_t axis = 0; axis < _AXIS_NUM; axis++)
    {
        _setAngles[axis] = 0;
        _resetAngles[axis] = 0;
    }

    //Initialize index translator
    _index = Utilities::find(address, _sensorAddrs, _sensorNum);
}

void IMU_Wrapper::setOffsets(const adafruit_bno055_offsets_t &offsets)
{
    //Check if sensor already has offsets
    if (!_imusPtr[_index]->haveOffsets())
    {
        //If not then set new offsets
        _imusPtr[_index]->setOffsets(offsets);
    }
}

void IMU_Wrapper::begin(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                        /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    //Check if sensor has already begun
    if (!_haveBegun[_index])
    {
        //If not, begin sensor
        _imusPtr[_index]->begin(mode);

        //Indicate that sensor has begun
        _haveBegun[_index] = true;
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::beginWithoutOffsets(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                                      /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    //Check if sensor has already begun
    if (!_haveBegun[_index])
    {
        //If not, begin sensor
        _imusPtr[_index]->beginWithoutOffsets(mode);

        //Indicate that sensor has begun
        _haveBegun[_index] = true;
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::setExtCrystalUse(bool usextal)
{
    //Check if crystal has already been set
    if (!_haveSetCrystal[_index])
    {
        //If not, set crystal
        _imusPtr[_index]->setExtCrystalUse(usextal);

        //Indicate that crystal has been set
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::update(bool offsetRegen /*= true*/)
{
    //Update sensor
    _imusPtr[_index]->update(offsetRegen);
}

void IMU_Wrapper::reset(double yaw /*= 0*/, double pitch /*= 0*/, double roll /*= 0*/)
{
    //Save yaw angle to set
    _setAngles[_YAW_INDEX] = yaw;

    //Save cur yaw angle for resetting
    _resetAngles[_YAW_INDEX] = _imusPtr[_index]->getYaw();

    //Save pitch angle to set
    _setAngles[_PITCH_INDEX] = pitch;

    //Save cur pitch angle for resetting
    _resetAngles[_PITCH_INDEX] = _imusPtr[_index]->getPitch();

    //Save roll angle to set
    _setAngles[_ROLL_INDEX] = roll;

    //Save cur roll angle for resetting
    _resetAngles[_ROLL_INDEX] = _imusPtr[_index]->getRoll();
}

void IMU_Wrapper::resetSensor()
{
    //Check for only one instance
    if (_instanceNum == 1)
    {
        //Reset the sensor
        _imusPtr[_index]->resetSensor();

        //Indicate that crystal needs to be reset
        _haveSetCrystal[_index] = false;
    }
}

void IMU_Wrapper::displayOffsets(const adafruit_bno055_offsets_t &calibData)
{
    //Display given offsets
    IMU::displayOffsets(calibData);
}

void IMU_Wrapper::displayOffsets()
{
    //Check if offsets have been given
    if (_imusPtr[_index]->haveOffsets())
    {
        //Display given offsetss
        displayOffsets(_imusPtr[_index]->getOffsets());
    }
    //Offsets have not been given
    else
    {
        //Get offsets and display them
        _imusPtr[_index]->displayOffsets();
    }
}

void IMU_Wrapper::displaySensorDetails()
{
    //Display sensor details
    _imusPtr[_index]->displaySensorDetails();
}

void IMU_Wrapper::displaySensorStatus()
{
    //Display sensor status
    _imusPtr[_index]->displaySensorStatus();
}

void IMU_Wrapper::displayCalStatus()
{
    //Display calibration status
    _imusPtr[_index]->displayCalStatus();
}

void IMU_Wrapper::displayOrientation()
{
    //Display orientation data
    Serial.print("Yaw: ");
    Serial.print(getYaw(), 4);
    Serial.print("\tPitch: ");
    Serial.print(getPitch(), 4);
    Serial.print("\tRoll: ");
    Serial.print(getRoll(), 4);
}

double IMU_Wrapper::getYaw()
{
    //Display yaw after setting and resetting
    return _imusPtr[_index]->getYaw() - _resetAngles[_YAW_INDEX] + _setAngles[_YAW_INDEX];
}

double IMU_Wrapper::getPitch()
{
    //Display pitch after setting and resetting
    return _imusPtr[_index]->getPitch() - _resetAngles[_PITCH_INDEX] - _setAngles[_PITCH_INDEX];
}

double IMU_Wrapper::getRoll()
{
    //Display roll after setting and resetting
    return _imusPtr[_index]->getRoll() - _resetAngles[_ROLL_INDEX] - _setAngles[_ROLL_INDEX];
}

adafruit_bno055_offsets_t IMU_Wrapper::getSensorOffsets()
{
    //Return sensor offsets directly from sensor
    return _imusPtr[_index]->getSensorOffsets();
}

adafruit_bno055_offsets_t IMU_Wrapper::generateOffsets()
{
    //Generate offsets and then return them
    return _imusPtr[_index]->generateOffsets();
}

adafruit_bno055_offsets_t IMU_Wrapper::getOffsets()
{
    //Return given offsets
    return _imusPtr[_index]->getOffsets();
}

unsigned int IMU_Wrapper::getRST()
{
    //Return reset pin
    return _imusPtr[_index]->getRST();
}

unsigned int IMU_Wrapper::getSensorAddr()
{
    //Return sensor address
    return _sensorAddrs[_index];
}

size_t IMU_Wrapper::getTotalSensorNum()
{
    //Return the number of sensor initialized
    return _sensorNum;
}

bool IMU_Wrapper::isFullyCalibrated()
{
    //Return true if sensor is fully calibrated
    return _imusPtr[_index]->isFullyCalibrated();
}

bool IMU_Wrapper::haveOffsets()
{
    //Return true if sensor has been given offsets
    return _imusPtr[_index]->haveOffsets();
}