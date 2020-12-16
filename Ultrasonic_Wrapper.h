#pragma once
#include <Arduino.h>
#include "Ultrasonic.h"

class Ultrasonic_Wrapper
{
    private:
        const unsigned int _trigPin;

        const size_t _sensorNum;

        Ultrasonic **_ultrasonicsPtr;

    public:
        static const size_t ULTRASONIC_FRONT = 0;
        static const size_t ULTRASONIC_LEFT = 1;
        static const size_t ULTRASONIC_RIGHT = 2;
        static const size_t ULTRASONIC_BACK = 3;

    public:
        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int* burstFrequencies,
                           size_t sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int burstFrequency,
                           size_t sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int echoPin,
                           unsigned int burstFrequency);

        ~Ultrasonic_Wrapper();

        void begin(void (*externalEchoPinISRs[])());

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance(size_t index = 0) const;

        unsigned int getEchoPin(size_t index = 0) const;

        unsigned int getTrigPin() const;

        void echoPinISR(size_t index = 0);
    
    private:
        void _init(unsigned int *echoPins, unsigned int *burstFrequencies);
};