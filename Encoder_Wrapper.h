#pragma once
#include <Arduino.h>
#include "Encoder.h"

class Encoder_Wrapper
{
    private:
        static unsigned int _instanceNum;
        static const unsigned int _pinsPerSensor = 2;

        static size_t _totalSensorNum;
        size_t _sensorNum = -1;
        static Encoder **_encodersPtr;
        static unsigned int *_pins;

        long int *_resetCounts;
        long int *_setCounts;
        size_t *_indices;

    public:
        Encoder_Wrapper(unsigned int* pins, size_t sensorNum);

        ~Encoder_Wrapper();

        void setCount(long int newCount, size_t sensor = 0);

        void resetCount(size_t sensor = 0xFFFFFFFF);    //value = -1

        long int getCount(size_t sensor = 0);

        unsigned int getPin(size_t sensor = 0, size_t index = 0);
    
    private:
        static size_t _find(unsigned int *newPins, size_t newSensorIndex,
                            unsigned int *oldPins, size_t oldSensorNum);
        static size_t _find(size_t newPinIndex, size_t *oldPinIndices,
                            size_t oldPinIndicesNum);
};