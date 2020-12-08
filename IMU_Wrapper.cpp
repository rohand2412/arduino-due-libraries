#include "IMU_Wrapper.h"

IMU_Wrapper::IMU_Wrapper(unsigned int mpuInterruptPin) : IMU(mpuInterruptPin)
{
    _oldYawRaw = 0;
    _oldPitchRaw = 0;
    _oldRollRaw = 0;
    _yaw = 0;
    _pitch = 0;
    _roll = 0;
}

void IMU_Wrapper::update()
{
  updateRaw();
  float yawRaw = getYawRaw();
  float pitchRaw = getPitchRaw();
  float rollRaw = getRollRaw();
  _overflow(_oldYawRaw, yawRaw, _yaw);
  _overflow(_oldPitchRaw, pitchRaw, _pitch);
  _overflow(_oldRollRaw, rollRaw, _roll);
}

void IMU_Wrapper::reset(float yaw /*=0*/, float pitch /*=0*/, float roll /*=0*/)
{
  _yaw = yaw;
  _pitch = pitch;
  _roll = roll;
}

float IMU_Wrapper::getYaw() const
{
  return _yaw;
}

float IMU_Wrapper::getPitch() const
{
  return _pitch;
}

float IMU_Wrapper::getRoll() const
{
  return _roll;
}

void IMU_Wrapper::_overflow(float& oldRaw, float& raw, float& axis)
{
  float threshold = 300; //Num of deg at which event is considered overflow
  float fullCircleDeg = 360; //Num of deg in a circle
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