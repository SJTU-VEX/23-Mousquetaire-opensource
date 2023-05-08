#include "my-timer.h"
#include "vex.h"

MyTimer::MyTimer()
{
    startTime = Brain.Timer.value();
}

void MyTimer::reset()
{
    startTime = Brain.Timer.value();
}

int MyTimer::getTime() const
{
    return floor((Brain.Timer.value() - startTime) * 1000); // return time (msec) from startTime
}

double MyTimer::getTimeDouble() const
{
    return Brain.Timer.value() - startTime; // return time (sec) from startTime
}