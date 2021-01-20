#include <Arduino.h>
#include "IMU.h"

IMU::IMU(unsigned int RST, unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/) : _RST(RST), _sensorID(sensorID), _address(address)
{
    //Initialize Adafruit Sensor object
    _bno = new Adafruit_BNO055(sensorID, address);
}

IMU::~IMU()
{
    //Destroy Adafruit sensor object
    delete _bno;
}

void IMU::setOffsets(const adafruit_bno055_offsets_t &offsets)
{
    //Store offsets
    _offsets = offsets;

    //Indicate that offsets have been given
    _haveOffsets = true;
}

void IMU::begin(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                        /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    //Store sensor mode
    _mode = mode;

    //Configure reset pin
    pinMode(_RST, OUTPUT);

    //Reset sensor just in case
    resetSensor();

    //Check if connection was successful
    //as well as establish connection
    if (!_bno->begin(mode))
    {
        //Indicate to user that connection was unsuccessful
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    //Check if offsets have been given
    if (_haveOffsets)
    {
        //Set offsets to sensor
        _bno->setSensorOffsets(_offsets);
    }
    //Offsets have not been given
    else
    {
        //Make sure user cannot miss message
        while (true)
        {
            //Indication for user that offsets need to be given
            Serial.println("[ERROR] NO OFFSETS WERE GIVEN FOR IMU");
        }
    }

    //Tell sensor to use external sensor
    //and complete configuration
    //(so only call after config is complete)
    _bno->setExtCrystalUse(true);

    //Wait for offsets to take full effect
    while (!isFullyCalibrated())
    {
        //Keep updating sensor data
        update(false);
    }

    //Final update of sensor data before
    //initialization is declared complete
    update();
}

void IMU::beginWithoutOffsets(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                                      /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS*/)
{
    //Store sensor mode
    _mode = mode;

    //Configure reset pin
    pinMode(_RST, OUTPUT);

    //Reset sensor just in case
    resetSensor();

    //Check if connection was successful
    //as well as establish connection
    if (!_bno->begin(mode))
    {
        //Indicate to user that connection was unsuccessful
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }

    //Tell sensor to use external sensor
    //and complete configuration
    //(so only call after config is complete)
    _bno->setExtCrystalUse(true);

    //Final update of sensor data before
    //initialization is declared complete
    update();
}

void IMU::setExtCrystalUse(bool usextal)
{
    //Set sensor usage of external crystal
    //as well as complete sensor configuration
    _bno->setExtCrystalUse(usextal);
}

void IMU::update(bool offsetRegen /*= true*/)
{
    //Check if sampling rate delay has passed
    if (millis() - _lastUpdated_MS >= _BNO055_SAMPLERATE_DELAY_MS)
    {
        //Poll sensor for new sensor info
        _bno->getSensor(&_sensor);

        //Reset calibration data
        _systemCal = 0;
        _gyroCal = 0;
        _accelCal = 0;
        _magCal = 0;

        //Poll sensor for new calibration data
        _bno->getCalibration(&_systemCal, &_gyroCal, &_accelCal, &_magCal);

        //Check if calibration is inaccurate
        //and if user has allowed regen of offsets
        //and if instance has offsets to regen with
        if (!isFullyCalibrated() && offsetRegen && _haveOffsets)
        {
            //Get latest offsets from sensor
            adafruit_bno055_offsets_t curOffsets = getSensorOffsets();

            //Check if gyro is not calibrated
            if (_gyroCal != 3)
            {
                //Repair gyro offsets
                curOffsets.gyro_offset_x = _offsets.gyro_offset_x;
                curOffsets.gyro_offset_y = _offsets.gyro_offset_y;
                curOffsets.gyro_offset_z = _offsets.gyro_offset_z;
            }
            //Check if accel is not calibrated
            if (_accelCal != 3)
            {
                //Repair accel offsets
                curOffsets.accel_offset_x = _offsets.accel_offset_x;
                curOffsets.accel_offset_y = _offsets.accel_offset_y;
                curOffsets.accel_offset_z = _offsets.accel_offset_z;
                curOffsets.accel_radius = _offsets.accel_radius;
            }
            //Check if mag is not calibrated
            if (_magCal != 3)
            {
                //Repair mag offsets
                curOffsets.mag_offset_x = _offsets.mag_offset_x;
                curOffsets.mag_offset_y = _offsets.mag_offset_y;
                curOffsets.mag_offset_z = _offsets.mag_offset_z;
                curOffsets.mag_radius = _offsets.mag_radius;
            }

            //Check if mag or system is not calibrated
            if (_magCal != 3 || _systemCal != 3)
            {
                //Monitor number of times mag fails
                _badMagCounter++;
            }

            //Set operation mode to configuration for setting offsets
            _bno->setMode(Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_CONFIG);

            //Set repaired offsets
            _bno->setSensorOffsets(curOffsets);

            //If mag has gone bad more than or equal to preset limit
            if (!(_badMagCounter < _BAD_MAG_MAX))
            {
                //Ditch mag and change mode to just accel and gyro
                _mode = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS;
            }

            //Set new or old mode back from config
            _bno->setMode(_mode);
        }

        //Check if system is calibrated and if mode is NDOF or just if mode is IMUPLUS
        if ((_systemCal == 3 && _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF)
            || _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS)
        {
            //Get new event
            _bno->getEvent(&_event);

            //Store orientation data
            double yawRaw = _event.orientation.x;
            double pitchRaw = _event.orientation.y;
            double rollRaw = _event.orientation.z;

            //Account for axis wrap around/overflow
            //and update instance data
            _overflow(_oldYawRaw, yawRaw, _yaw);
            _overflow(_oldPitchRaw, pitchRaw, _pitch);
            _overflow(_oldRollRaw, rollRaw, _roll);
        }

        //Store when update was last called
        _lastUpdated_MS = millis();
    }
}

void IMU::reset(double yaw /*=0*/, double pitch /*=0*/, double roll /*=0*/)
{
    //Software reset angle values
    _yaw = yaw;
    _pitch = pitch;
    _roll = roll;
}

void IMU::resetSensor()
{
    //Set reset pin LOW
    digitalWrite(_RST, LOW);

    //Wait for sensor to acknowledge
    //that reset is being initiated
    delay(1);

    //Put reset pin back to HIGH
    digitalWrite(_RST, HIGH);
}

void IMU::displayOffsets(const adafruit_bno055_offsets_t &calibData)
{
    //Display accel data
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_y);
    Serial.print(" ");
    Serial.print(calibData.accel_offset_z);
    Serial.print(" ");

    //Display gyro data
    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_y);
    Serial.print(" ");
    Serial.print(calibData.gyro_offset_z);
    Serial.print(" ");

    //Display mag data
    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_y);
    Serial.print(" ");
    Serial.print(calibData.mag_offset_z);
    Serial.print(" ");

    //Display accel radius
    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    //Display mag radius
    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);

    //New line
    Serial.print("\n");
}

void IMU::displayOffsets()
{
    //Get offsets and then display them
    displayOffsets(getOffsets());
}

void IMU::displaySensorDetails()
{
    //Effectively clear terminal
    Serial.println("------------------------------------");

    //Display sensor hardware details
    Serial.print("Sensor:       ");
    Serial.println(_sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(_sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(_sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(_sensor.max_value);
    Serial.println(" xxx");
    Serial.print("Min Value:    ");
    Serial.print(_sensor.min_value);
    Serial.println(" xxx");
    Serial.print("Resolution:   ");
    Serial.print(_sensor.resolution);
    Serial.println(" xxx");

    //Clear terminal again
    Serial.println("------------------------------------");
    Serial.println("");
}

void IMU::displaySensorStatus()
{
    //Initialize status vars
    uint8_t system_status = 0;
    uint8_t self_test_results = 0;
    uint8_t system_error = 0;

    //Poll sensor for status data
    _bno->getSystemStatus(&system_status, &self_test_results, &system_error);

    //Display newly acquired data
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
}

void IMU::displayCalStatus()
{
    //Create space between other data being displayed
    Serial.print("\t");

    //Check if system calibration is 0
    if (!_systemCal)
    {
        //Indicate to user that sensor is 
        //recommended not use in when not calibrated
        Serial.print("! ");
    }

    //Display remaining calibration data
    Serial.print("Sys:");
    Serial.print(_systemCal, DEC);
    Serial.print(" G:");
    Serial.print(_gyroCal, DEC);
    Serial.print(" A:");
    Serial.print(_accelCal, DEC);
    Serial.print(" M:");
    Serial.print(_magCal, DEC);

    //Display number of times mag has failed so far
    Serial.print(" BMG:");
    Serial.print(_badMagCounter, DEC);

    //Display current mode sensor is in
    Serial.print(" M:");
    Serial.print(_mode, HEX);

    //New line
    Serial.print("\n");
}

void IMU::displayOrientation()
{
    //Display orientation data
    Serial.print("Yaw: ");
    Serial.print(getYaw(), 4);
    Serial.print("\tPitch: ");
    Serial.print(getPitch(), 4);
    Serial.print("\tRoll: ");
    Serial.print(getRoll(), 4);
}

double IMU::getYaw()
{
    //Check if mag is calibrated and mode is NDOF or just mode is IMUPLUS
    if ((_magCal == 3 && _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF)
        || _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS)
    {
        //Only return latest data if true
        _returnYaw = _yaw;
    }

    //Return yaw
    return _returnYaw;
}

double IMU::getPitch()
{
    //Check if mag is calibrated and mode is NDOF or just mode is IMUPLUS
    if ((_magCal == 3 && _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF)
        || _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS)
    {
        //Only return latest data if true
        _returnPitch = _pitch;
    }

    //Return pitch
    return _returnPitch;
}

double IMU::getRoll()
{
    //Check if mag is calibrated and mode is NDOF or just mode is IMUPLUS
    if ((_magCal == 3 && _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF)
        || _mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS)
    {
        //Only return latest data if true
        _returnRoll = _roll;
    }

    //Return roll
    return _returnRoll;
}

adafruit_bno055_offsets_t IMU::getSensorOffsets()
{
    //Initialize offsets var
    adafruit_bno055_offsets_t offsets;

    //Poll sensor for latest offsets
    _bno->getSensorOffsets(offsets);

    //Return latest offsets
    return offsets;
}

adafruit_bno055_offsets_t IMU::generateOffsets()
{
    //Clear current calibration data
    _systemCal = 0;
    _gyroCal = 0;
    _accelCal = 0;
    _magCal = 0;

    //Repeat until sensor is fully calibrated
    while (!isFullyCalibrated())
    {
        //Poll sensor for new data
        update();

        //Display orientation data
        displayOrientation();

        //Display calibration status
        displayCalStatus();
    }

    //Indicate to user that calibration is complete
    Serial.println("\nFully calibrated!");

    //Initialize offset var
    adafruit_bno055_offsets_t newCalib;

    //Fetch fully calibrated results
    _bno->getSensorOffsets(newCalib);

    //Return fully calibrated offsets
    return newCalib;
}

adafruit_bno055_offsets_t IMU::getOffsets()
{
    //Check if instance has offsets
    if (_haveOffsets)
    {
        //Return offsets previously given
        return _offsets;
    }
    //Instance does not have offsets
    else
    {
        //Make sure user cannot miss message
        while (true)
        {
            //Indication for user that offsets need to be given
            Serial.println("[ERROR] NO OFFSETS WERE GIVEN FOR IMU");
        }
    }
}

unsigned int IMU::getRST()
{
    //Return reset pin
    return _RST;
}

unsigned int IMU::getSensorID()
{
    //Return sensor ID
    return _sensorID;
}

unsigned int IMU::getAddress()
{
    //Return I2C address of the sensor
    return _address;
}

void IMU::_overflow(double &oldRaw, double &raw, double &axis)
{
    //Num of deg at which delta is considered overflow
    double threshold = 300;

    //Num of deg in a circle
    double fullCircleDeg = 360;

    //Check if delta raw readings is greater than threshold
    if (raw - oldRaw > threshold)
    {
        //Detect and revert overflow
        axis -= fullCircleDeg;
    }
    //Check if delta raw readings is less than negative threshold
    else if (raw - oldRaw < -threshold)
    {
        //Detect and revert overflow
        axis += fullCircleDeg;
    }

    //Add delta to current software sensor data
    axis += raw - oldRaw;

    //Save reading as old reading
    oldRaw = raw;
}

bool IMU::isFullyCalibrated()
{
    //Check if mode is IMUPLUS
    if (_mode == Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS)
    {
        //Only gyro and accel used in IMUPLUS
        return _gyroCal == 3 && _accelCal == 3;
    }

    //NDOF mode uses all sensors
    return _systemCal == 3 && _gyroCal == 3 && _accelCal == 3 && _magCal == 3;
}

bool IMU::haveOffsets()
{
    //Return whether instance has offsets or not
    return _haveOffsets;
}