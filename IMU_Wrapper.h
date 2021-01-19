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
        double _returnYaw = 0;
        double _returnPitch = 0;
        double _returnRoll = 0;

        const unsigned int _BNO055_SAMPLERATE_DELAY_MS = 100;
        unsigned int _lastUpdated_MS;

        const unsigned int _RST;

        Adafruit_BNO055::adafruit_bno055_opmode_t _mode;
        adafruit_bno055_offsets_t _offsets;
        bool _haveOffsets = false;

        unsigned int _badMagCounter = 0;
        const unsigned int _BAD_MAG_MAX = 10;

    public:
        IMU_Wrapper(unsigned int RST, unsigned int sensorID = 55, uint8_t address = 0x28);

        ~IMU_Wrapper();

        void setOffsets(const adafruit_bno055_offsets_t &offsets);

        void begin(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

        void beginWithoutOffsets(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

        void setExtCrystalUse(bool usextal);

        void update(bool offsetRegen = true);

        void reset(double yaw = 0, double pitch = 0, double roll = 0);

        void resetSensor();

        static void displayOffsets(const adafruit_bno055_offsets_t &calibData);

        void displayOffsets();

        void displaySensorDetails();

        void displaySensorStatus();

        void displayCalStatus();

        void displayOrientation();

        double getYaw();

        double getPitch();

        double getRoll();

        adafruit_bno055_offsets_t getSensorOffsets();

        adafruit_bno055_offsets_t getOffsets();

        bool isFullyCalibrated();

    private:
        void _overflow(double& oldRaw, double& raw, double& axis);
};