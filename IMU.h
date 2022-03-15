/**
 * Wraps Adafruit's BNO055 drivers and reduces them to some convenient accessor methods
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
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>

class IMU
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
        const unsigned int _sensorID;
        const uint8_t _address;

        Adafruit_BNO055::adafruit_bno055_opmode_t _mode;
        adafruit_bno055_offsets_t _offsets;
        bool _haveOffsets = false;

        unsigned int _badMagCounter = 0;
        const unsigned int _BAD_MAG_MAX = 10;

    public:
        IMU(unsigned int RST, unsigned int sensorID = 55, uint8_t address = 0x28);

        ~IMU();

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

        unsigned int getAddress();

        bool isFullyCalibrated();

        bool haveOffsets();

    private:
        void _overflow(double& oldRaw, double& raw, double& axis);
};