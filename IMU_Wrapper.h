/**
 * Manages universal IMU objects that IMU_Wrapper instances fetch from
 * Copyright (C) 2022  Rohan Dugad
 * 
 * Contact info:
 * https://docs.google.com/document/d/17IhBs4cz7FXphE0praCaWMjz016a7BFU5IQbm1CNnUc/edit?usp=sharing
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <Arduino.h>
#include "IMU.h"

class IMU_Wrapper
{
    private:
        static size_t _instanceNum;
        static size_t _sensorNum;

        static IMU **_imusPtr;
        static unsigned int *_sensorAddrs;
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

        unsigned int getSensorAddr();

        static size_t getTotalSensorNum();

        bool isFullyCalibrated();

        bool haveOffsets();
};