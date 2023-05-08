#ifndef ADJUSTMENT_H_
#define ADJUSTMENT_H_
#include "PID.h"

void autoSpinRadiusCW();
void autoSpinRadiusCC();
void adjustForward(float _power);
float adjustGetPos();
void testFlyWheel();
void turnToLock(double _tarAng);

#endif