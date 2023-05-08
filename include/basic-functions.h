#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_


void delay(int msec);

// Basic movement
void moveClockwise(float percent);
void moveIntaker(float percent);
void moveIntakerWithRPM(float RPM);

double IMUHeading();
void setIMUHeading(double degree);

void clearBrainScr();
void clearControllerScr();

// Output functions
// void setPiston_Deploy(bool);

#endif