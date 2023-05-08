#ifndef AUTON_FUNCTIONS_H_
#define AUTON_FUNCTIONS_H_
#include "geometry.h"
int autonAiming();


void timerForwardWithHeading(float _power, int _duration, double _targetHeading);
void timerMove(Vector vel, int msec);

void moveToWithHeading(double _x, double _y, double _tarAng, double _maxSpeed);
void quickMoveToWithHeading(double _x, double _y, double _tarAng, double _maxSpeed);
void quickMoveToWithoutStop(double _x, double _y, double _tarAng, double _maxSpeed);

void aimAt(double _x, double _y, double _offset = 0);
void aimPreciselyAt(double _x, double _y, double _offset = 0);
void turnTo(double _tarAng, double _maxSpeed = 50);
void quickTurnTo(double _tarAng, double _maxSpeed = 50);

void shoot(int _num = 1, int _msec = 0);
void turnRoller();

void intake3DiscsWithPiston(double _x, double _y, double _ang);
void intake3DiscsWithoutPiston(double _x, double _y, double _ang);




#endif