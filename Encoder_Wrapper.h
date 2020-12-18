#pragma once
#include <Arduino.h>
#include "Encoder.h"

class Encoder_Wrapper
{
    private:
        static unsigned int _instanceNum;
        static const unsigned int _pinsPerSensor = 2;

        static size_t _sensorNum;
        static Encoder **_encodersPtr;
        static unsigned int *_pins;

        long int *_resetCounts;
        long int *_setCounts;

    public:
        Encoder_Wrapper(unsigned int* pins, size_t sensorNum);

        ~Encoder_Wrapper();

        void setCount(long int newCount, size_t sensor = 0);

        void resetCount(size_t sensor = 1000);

        long int getCount(size_t sensor = 0);

        unsigned int getPin(size_t sensor = 0, size_t index = 0);
};