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
  unsigned int sum = 0;
  for (size_t item = startLen; item < endLen; item++)
  {
    sum += buffer[item];
  }
  return sum / (endLen - startLen);
}