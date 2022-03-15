/**
 * Collection of miscellaneous utility methods
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

class Utilities
{
    public:
        static size_t indexCap(size_t index, size_t maxIndex);

        static double calculatePid(double error, double derivative, double integral,
                                   double kp, double ki, double kd);

        static bool isEqual_DBL(double num, double target);

        static unsigned int average(unsigned int* buffer, size_t endLen, size_t startLen = 0);

        static size_t find(unsigned int &newNums, size_t newNumLen,
                           unsigned int *oldNums, size_t oldNumLen);

        static size_t find(unsigned int newNum,
                           unsigned int *oldNums, size_t oldNumLen);

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