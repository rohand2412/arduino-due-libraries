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