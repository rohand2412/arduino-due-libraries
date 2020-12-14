#pragma once
#include <Arduino.h>
#include "Ultrasonic.h"

class Ultrasonic_Wrapper
{
    private:
        const unsigned int _trigPin;

        const unsigned int _sensorNum;

        Ultrasonic *_ultrasonics;

    public:
        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int* burstFrequencies,
                           unsigned int sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int* echoPins,
                           unsigned int burstFrequency,
                           unsigned int sensorNum);

        Ultrasonic_Wrapper(unsigned int trigPin,
                           unsigned int echoPin,
                           unsigned int burstFrequency);

        void begin(void (*externalEchoPinISRs[])());

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance(unsigned int index = 0) const;

        unsigned int getEchoPin(unsigned int index = 0) const;

        unsigned int getTrigPin() const;

        void echoPinISR(unsigned int index = 0);
};