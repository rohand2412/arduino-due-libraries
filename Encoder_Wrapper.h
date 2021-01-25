#pragma once
#include <Arduino.h>
#include "Encoder.h"

class Encoder_Wrapper
{
    private:
        static unsigned int _instanceNum;

        static size_t _totalSensorNum;
        size_t _sensorNum = -1;
        static Encoder **_encodersPtr;
        static unsigned int *_pins;
        static bool _createdSensor;

        long int *_resetCounts;
        long int *_setCounts;
        size_t *_indices;
        size_t *_pinIndices;

    public:
        static const size_t PINS_PER_SENSOR = 2;

        static const size_t ENCODER_LEFT = 0;
        static const size_t ENCODER_RIGHT = 1;
        static const size_t ENCODER_OUT_A = 0;
        static const size_t ENCODER_OUT_B = 1;

    public:
        Encoder_Wrapper();

        Encoder_Wrapper(unsigned int* pins, size_t sensorNum = 1);

        ~Encoder_Wrapper();

        void begin(unsigned int *pins, size_t sensorNum = 1);

        void setCount(size_t sensor, long int newCount); //No default param in order to preserve order of params

        void resetCount(size_t sensor = 0xFFFFFFFF);    //sensor = -1

        long int getCount(size_t sensor = ENCODER_LEFT);

        unsigned int getPin(size_t sensor = ENCODER_LEFT, size_t index = ENCODER_OUT_A) const;

        size_t getSensorNum() const;

        static size_t getTotalSensorNum();

    private:
        static void _construct(unsigned int *pins, size_t newSensorNum,
                               size_t oldSensorNum);

        static void _construct(unsigned int *pins, size_t sensorNum);
};