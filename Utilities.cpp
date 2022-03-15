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

#include <Arduino.h>
#include "Utilities.h"

Utilities::Utilities(){};

size_t Utilities::indexCap(size_t index, size_t maxIndex)
{
    //Indices start at 0 so minus 1 is max index value
    maxIndex -= 1;

    //Make sure index is positive
    index = (index < 0) ? 0 : index;

    //Make sure index doesn't exceed maxIndex
    index = (index >= maxIndex) ? maxIndex : index;

    //Return index
    return index;
}

double Utilities::calculatePid(double error, double derivative, double integral,
                               double kp, double ki, double kd)
{
    //Declare result var
    double result;

    //Check if integral coefficient is not 0
    if (!isEqual_DBL(ki, 0))
    {
        //Apply PID formula
        result = kp *
                 (error
                 + (1 / ki) * integral
                 + kd * derivative);
    }
    //integral coefficient is 0
    else
    {
        //Apply PD formula
        result = kp *
                 (error
                 + kd * derivative);
    }

    //Return result
    return result;
}

bool Utilities::isEqual_DBL(double num, double target)
{
    //Check if num is near target by __DBL_EPSILON__
    //Check is necessary due to floating point precision
    if (fabs(num - target) <= __DBL_EPSILON__)
    {
        //true if near
        return true;
    }
    //Otherwise false
    return false;
}

unsigned int Utilities::average(unsigned int* buffer, size_t endLen, size_t startLen /*= 0*/)
{
    //Initialize sum
    unsigned int sum = 0;

    //Add up all values
    for (size_t item = startLen; item < endLen; item++)
    {
        sum += buffer[item];
    }

    //Divide by num of items to get average
    return sum / (endLen - startLen);
}

size_t Utilities::find(unsigned int &newNums, size_t newNumLen,
                       unsigned int *oldNums, size_t oldNumLen)
{
    //Convert first num to num array
    unsigned int *newNumsPtr = &newNums;

    //Maps new num indices to old num indices
    size_t numIndices[newNumLen];

    //Iterate through nums
    for (size_t num = 0; num < newNumLen; num++)
    {
        //Populate numIndices
        numIndices[num] = find(newNumsPtr[num], oldNums, oldNumLen);

        //Check if any of the nums weren't found
        if (numIndices[num] == 0xFFFFFFFF) //== -1
        {
            //Return -1 if no match found
            return 0xFFFFFFFF;
        }
    }

    //Num Index is arbitrary
    //Int truncation will make result equivalent
    //to old num group index
    return numIndices[0] / newNumLen;
}

size_t Utilities::find(unsigned int newNum,
                       unsigned int *oldNums, size_t oldNumLen)
{
    //Iterate through old nums
    for (size_t oldNum = 0; oldNum < oldNumLen; oldNum++)
    {
        //Check if new num matches old num iteration
        if (oldNums[oldNum] == newNum)
        {
            //Return running iterator if match found
            return oldNum;
        }
    }

    //Return -1 if no match found
    return 0xFFFFFFFF;
}

AverageSign::AverageSign(size_t len)
{
    //Store num of items
    _len = len;

    //Allocate memory needed
    _ints = new unsigned int[_len];
    _signs = new bool[_len];

    //Populate memory
    for (size_t num = 0; num < _len; num++)
    {
        _ints[num] = 0;
        _signs[num] = 1;
    }
}

void AverageSign::setInt(int val)
{
    //Store absolute value of value
    _ints[_index] = abs(val);

    //Store sign
    _signs[_index] = !(val < 0);

    //Increment for next index
    _index++;
}

int AverageSign::getAverage()
{
    //Get average of absolute values
    int avg = Utilities::average(_ints, _len);

    //Initialize num of negative nums
    unsigned int numNeg = 0;

    //Iterate through numbers
    for (size_t num = 0; num < _len; num++)
    {
        //Count how many negative nums
        numNeg += !_signs[num];
    }

    //If more than half of items are neg
    if (numNeg > _len / 2)
    {
        //Return negative avg
        return avg * -1;
    }
    //More than half of items are pos
    else
    {
        //Return positive avg
        return avg;
    }
}