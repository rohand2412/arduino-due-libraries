#pragma once
#include <Arduino.h>

class Utilities
{
    public:
        static size_t indexCap(size_t index, size_t maxIndex);

        static double calculatePid(double error, double derivative, double integral,
                                   double kp, double ki, double kd);

        static bool isEqual_DBL(double num, double target);

        static unsigned int average(unsigned int* buffer, size_t endLen, size_t startLen = 0);

    private:
        Utilities();
};

class AverageSign
{
    private:
        unsigned int *_ints;
        bool *_signs;
        size_t _len;
        size_t _index = 0;

    public:
        AverageSign(size_t len);

        void setInt(int val);

        int getAverage();
};