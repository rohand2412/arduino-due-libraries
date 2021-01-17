#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>

class IMU_Wrapper
{
    private:
        Adafruit_BNO055 *_bno;
        sensor_t _sensor;
        sensors_event_t _event;
        uint8_t _system_status = 0;
        uint8_t _self_test_results = 0;
        uint8_t _system_error = 0;
        uint8_t _systemCal = 0;
        uint8_t _gyroCal = 0;
        uint8_t _accelCal = 0;
        uint8_t _magCal = 0;

        double _oldYawRaw = 0;
        double _oldPitchRaw = 0;
        double _oldRollRaw = 0;
        double _yaw = 0;
        double _pitch = 0;
        double _roll = 0;

        const unsigned int _BNO055_SAMPLERATE_DELAY_MS = 100;

    public:
        IMU_Wrapper(unsigned int sensorID = 55, uint8_t address = 0x28);

        void begin(Adafruit_BNO055::adafruit_bno055_opmode_t mode = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_NDOF);

        void update();

        void reset(double yaw = 0, double pitch = 0, double roll = 0);

        void displaySensorDetails();

        void displaySensorStatus();

        void displayCalStatus();

        void displayOrientation();

        double getYaw() const;

        double getPitch() const;

        double getRoll() const;
    
    private:
        void _overflow(double& oldRaw, double& raw, double& axis);
};