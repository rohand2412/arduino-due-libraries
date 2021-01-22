#pragma once
#include <Arduino.h>
#include "IMU.h"

class IMU_Wrapper
{
    private:
        static size_t _instanceNum;
        static size_t _sensorNum;

        static IMU **_imusPtr;
        static unsigned int *_sensorIDs;
        static bool *_haveBegun;
        static bool *_haveSetCrystal;
        static bool _createdSensor;

        static const unsigned int _AXIS_NUM = 3;
        static const size_t _YAW_INDEX = 0;
        static const size_t _PITCH_INDEX = 1;
        static const size_t _ROLL_INDEX = 2;
        double _setAngles[_AXIS_NUM];
        double _resetAngles[_AXIS_NUM];

        size_t _index;

    public:
        IMU_Wrapper(unsigned int RST, unsigned int sensorID = 55, uint8_t address = 0x28);

        IMU_Wrapper();

        ~IMU_Wrapper();

        void createSensor(unsigned int RST, unsigned int sensorID = 55, uint8_t address = 0x28);

        void setOffsets(const adafruit_bno055_offsets_t &offsets);

        void begin(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                   = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

        void beginWithoutOffsets(const Adafruit_BNO055::adafruit_bno055_opmode_t &mode
                                 = Adafruit_BNO055::adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

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

        adafruit_bno055_offsets_t generateOffsets();

        adafruit_bno055_offsets_t getOffsets();

        unsigned int getRST();

        unsigned int getSensorID();

        static size_t getTotalSensorNum();

        bool isFullyCalibrated();

        bool haveOffsets();
};