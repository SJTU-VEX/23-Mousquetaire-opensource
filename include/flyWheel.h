#ifndef FLYWHEEL_H_
#define FLYWHEEL_H_
extern double FlyWheel_Error;
extern float FLYWHEEL_SPEED;
float const Flywheel_P_Control_Domain = 10;
float const kp = 80 / 10;
void setFlyWheelSpeed(double Target_Speed);
void updateFlyWheelSpeed(double Target_Speed);
double FlywheelVoltagetoVelocity(double percent);
double FlywheelVelocitytoVoltage(double speed);
void turnFlyWheelVoltage(float percent);
void FlyWheelSpeed();
void updateFlyWheel();
#endif