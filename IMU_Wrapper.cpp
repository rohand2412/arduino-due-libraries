#include <Arduino.h>
#include "IMU_Wrapper.h"
#include "Utilities.h"

size_t IMU_Wrapper::_instanceNum = 0;

size_t IMU_Wrapper::_sensorNum = 0;

IMU **IMU_Wrapper::_imusPtr;

unsigned int *IMU_Wrapper::_sensorIDs;

bool *IMU_Wrapper::_haveBegun;

bool *IMU_Wrapper::_haveSetCrystal;

bool IMU_Wrapper::_createdSensor = false;

IMU_Wrapper::IMU_Wrapper(unsigned int RST, unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/)
{
    _instanceNum++;

    createSensor(RST, sensorID, address);
}

IMU_Wrapper::IMU_Wrapper()
{
    _instanceNum++;
}

IMU_Wrapper::~IMU_Wrapper()
{
    if (_instanceNum == 1)
    {
        for (size_t sensor = 0; sensor < _sensorNum; sensor++)
        {
            delete _imusPtr[sensor];
        }
        _sensorNum = 0;
        _createdSensor = false;

        delete[] _imusPtr;
        delete[] _sensorIDs;
        delete[] _haveBegun;
        delete[] _haveSetCrystal;
    }

    _instanceNum--;
}

void IMU_Wrapper::createSensor(unsigned int RST, unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/)
{
    if (_instanceNum == 1)
    {
        if (!_createdSensor)
        {
            _sensorNum++;

            _imusPtr = new IMU *[_sensorNum];
            _sensorIDs = new unsigned int[_sensorNum];
            _haveBegun = new bool[_sensorNum];
            _haveSetCrystal = new bool[_sensorNum];

            for (size_t sensor = 0; sensor < _sensorNum; sensor++)
            {
                _imusPtr[sensor] = new IMU(RST, sensorID, address);
                _sensorIDs[sensor] = sensorID;
                _haveBegun[sensor] = false;
                _haveSetCrystal[sensor] = false;
            }

            _createdSensor = true;
        }
    }
    else if (Utilities::find(sensorID, _sensorIDs, _sensorNum) == 0xFFFFFFFF) //== -1
    {
        size_t oldSensorNum = _sensorNum;
        IMU **imusPtr = _imusPtr;
        unsigned int *sensorIDs = _sensorIDs;
        bool *haveBegun = _haveBegun;
        bool *haveSetCrystal = _haveSetCrystal;

        _sensorNum++;
        _imusPtr = new IMU *[_sensorNum];
        _sensorIDs = new unsigned int[_sensorNum];
        _haveBegun = new bool[_sensorNum];
        _haveSetCrystal = new bool[_sensorNum];

        for (size_t oldSensor = 0; oldSensor < oldSensorNum; oldSensor++)
        {
            _imusPtr[oldSensor] = imusPtr[oldSensor];
            _sensorIDs[oldSensor] = sensorIDs[oldSensor];
            _haveBegun[oldSensor] = haveBegun[oldSensor];
            _haveSetCrystal[oldSensor] = haveSetCrystal[oldSensor];
        }

        for (size_t newSensor = oldSensorNum; newSensor < _sensorNum; newSensor++)
        {
            _imusPtr[newSensor] = new IMU(RST, sensorID, address);
            _sensorIDs[newSensor] = sensorID;
            _haveBegun[newSensor] = false;
            _haveSetCrystal[newSensor] = false;
        }

        delete[] imusPtr;
        delete[] sensorIDs;
        delete[] haveBegun;
        delete[] haveSetCrystal;
    }

    for (size_t axis = 0; axis < _AXIS_NUM; axis++)
    {
        _setAngles[axis] = 0;
        _resetAngles[axis] = 0;
    }

    _index = Utilities::find(sensorID, _sensorIDs, _sensorNum);
}

void IMU_Wrapper::setOffsets(const adafruit_bno055_offsets_t &offsets)
{
    if (!_imusPtr[_index]->haveOffsets())
    {
        _imusPtr[_index]->setOffsets(offsets);
    }
}

void IMU_Wrapper::begin(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                        /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    if (!_haveBegun[_index])
    {
        _imusPtr[_index]->begin(mode);
        _haveBegun[_index] = true;
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::beginWithoutOffsets(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                                      /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    if (!_haveBegun[_index])
    {
        _imusPtr[_index]->beginWithoutOffsets(mode);
        _haveBegun[_index] = true;
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::setExtCrystalUse(bool usextal)
{
    if (!_haveSetCrystal[_index])
    {
        _imusPtr[_index]->setExtCrystalUse(usextal);
        _haveSetCrystal[_index] = true;
    }
}

void IMU_Wrapper::update(bool offsetRegen /*= true*/)
{
    _imusPtr[_index]->update(offsetRegen);
}

void IMU_Wrapper::reset(double yaw /*= 0*/, double pitch /*= 0*/, double roll /*= 0*/)
{
    _setAngles[_YAW_INDEX] = yaw;
    _resetAngles[_YAW_INDEX] = _imusPtr[_index]->getYaw();

    _setAngles[_PITCH_INDEX] = pitch;
    _resetAngles[_PITCH_INDEX] = _imusPtr[_index]->getPitch();

    _setAngles[_ROLL_INDEX] = roll;
    _resetAngles[_ROLL_INDEX] = _imusPtr[_index]->getRoll();
}

void IMU_Wrapper::resetSensor()
{
    if (_instanceNum == 1)
    {
        _imusPtr[_index]->resetSensor();
        _haveSetCrystal[_index] = false;
    }
}

void IMU_Wrapper::displayOffsets(const adafruit_bno055_offsets_t &calibData)
{
    IMU::displayOffsets(calibData);
}

void IMU_Wrapper::displayOffsets()
{
    _imusPtr[_index]->displayOffsets();
}

void IMU_Wrapper::displaySensorDetails()
{
    _imusPtr[_index]->displaySensorDetails();
}

void IMU_Wrapper::displaySensorStatus()
{
    _imusPtr[_index]->displaySensorStatus();
}

void IMU_Wrapper::displayCalStatus()
{
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
    return _imusPtr[_index]->getYaw() - _resetAngles[_YAW_INDEX] + _setAngles[_YAW_INDEX];
}

double IMU_Wrapper::getPitch()
{
    return _imusPtr[_index]->getPitch() - _resetAngles[_PITCH_INDEX] - _setAngles[_PITCH_INDEX];
}

double IMU_Wrapper::getRoll()
{
    return _imusPtr[_index]->getRoll() - _resetAngles[_ROLL_INDEX] - _setAngles[_ROLL_INDEX];
}

adafruit_bno055_offsets_t IMU_Wrapper::getSensorOffsets()
{
    return _imusPtr[_index]->getSensorOffsets();
}

adafruit_bno055_offsets_t IMU_Wrapper::generateOffsets()
{
    return _imusPtr[_index]->generateOffsets();
}

adafruit_bno055_offsets_t IMU_Wrapper::getOffsets()
{
    return _imusPtr[_index]->getOffsets();
}

unsigned int IMU_Wrapper::getRST()
{
    return _imusPtr[_index]->getRST();
}

unsigned int IMU_Wrapper::getSensorID()
{
    return _sensorIDs[_index];
}

size_t IMU_Wrapper::getTotalSensorNum()
{
    return _sensorNum;
}

bool IMU_Wrapper::isFullyCalibrated()
{
    return _imusPtr[_index]->isFullyCalibrated();
}

bool IMU_Wrapper::haveOffsets()
{
    return _imusPtr[_index]->haveOffsets();
}