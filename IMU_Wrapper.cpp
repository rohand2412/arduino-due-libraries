#include <Arduino.h>
#include "IMU_Wrapper.h"

IMU_Wrapper::IMU_Wrapper(unsigned int sensorID /*= 55*/, uint8_t address /*= 0x28*/)
{
  _bno = new Adafruit_BNO055(sensorID, address);
}

void IMU_Wrapper::begin(Adafruit_BNO055::adafruit_bno055_opmode_t mode /*= Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF*/)
{
  if(!_bno->begin(mode))
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  _bno->setExtCrystalUse(true);

  update();
}

void IMU_Wrapper::update()
{
  delay(_BNO055_SAMPLERATE_DELAY_MS);

  _bno->getSensor(&_sensor);

  _system_status = 0;
  _self_test_results = 0;
  _system_error = 0;
  _bno->getSystemStatus(&_system_status, &_self_test_results, &_system_error);

  _systemCal = 0;
  _gyroCal = 0;
  _accelCal = 0;
  _magCal = 0;
  _bno->getCalibration(&_systemCal, &_gyroCal, &_accelCal, &_magCal);

  _bno->getEvent(&_event);
  double yawRaw = _event.orientation.x;
  double pitchRaw = _event.orientation.y;
  double rollRaw = _event.orientation.z;
  _overflow(_oldYawRaw, yawRaw, _yaw);
  _overflow(_oldPitchRaw, pitchRaw, _pitch);
  _overflow(_oldRollRaw, rollRaw, _roll);
}

void IMU_Wrapper::reset(double yaw /*=0*/, double pitch /*=0*/, double roll /*=0*/)
{
  _yaw = yaw;
  _pitch = pitch;
  _roll = roll;
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void IMU_Wrapper::displaySensorDetails()
{
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(_sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(_sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(_sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(_sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(_sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(_sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void IMU_Wrapper::displaySensorStatus()
{
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(_system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(_self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(_system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void IMU_Wrapper::displayCalStatus()
{
  Serial.print("\t");
  if (!_systemCal)
  {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(_systemCal, DEC);
  Serial.print(" G:");
  Serial.print(_gyroCal, DEC);
  Serial.print(" A:");
  Serial.print(_accelCal, DEC);
  Serial.print(" M:");
  Serial.print(_magCal, DEC);
  Serial.print("\n");
}

void IMU_Wrapper::displayOrientation()
{
  Serial.print("Yaw: ");
  Serial.print(getYaw(), 4);
  Serial.print("\tPitch: ");
  Serial.print(getPitch(), 4);
  Serial.print("\tRoll: ");
  Serial.print(getRoll(), 4);
}

double IMU_Wrapper::getYaw() const
{
  return _yaw;
}

double IMU_Wrapper::getPitch() const
{
  return _pitch;
}

double IMU_Wrapper::getRoll() const
{
  return _roll;
}

void IMU_Wrapper::_overflow(double& oldRaw, double& raw, double& axis)
{
  double threshold = 300; //Num of deg at which event is considered overflow
  double fullCircleDeg = 360; //Num of deg in a circle
  if (raw - oldRaw > threshold)
  {
    axis -= fullCircleDeg;
  }
  else if (raw - oldRaw < -threshold)
  {
    axis += fullCircleDeg;
  }
  axis += raw - oldRaw;
  oldRaw = raw;
}