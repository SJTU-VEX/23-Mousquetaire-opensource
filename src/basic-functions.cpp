#include "basic-functions.h"
#include "vex.h"
#include "robot-config.h"
#include "chassis.h"
#include <iostream>
using namespace vex;

void delay(int msec) // 等待，单位毫秒
{
    this_thread::sleep_for(msec);
}

void moveClockwise(float percent)
{
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), percent);
}

void moveIntaker(float percent)
{
    Motor_Intaker1.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
    Motor_Intaker2.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
}

void moveIntakerWithRPM(float RPM)
{
    Motor_Intaker1.spin(directionType::fwd, RPM, velocityUnits::rpm);
    Motor_Intaker2.spin(directionType::fwd, RPM, velocityUnits::rpm);
}

/*????*/
double IMUHeading(){
    double heading = Inertial.rotation(rotationUnits::deg);
    heading = heading * IMUCoefficient;
    while (heading < 0) heading += 360;
    while (heading > 360) heading -= 360;
    return heading;
}

void setIMUHeading(double degree)
{
    Inertial.setRotation(degree/IMUCoefficient, rotationUnits::deg);
}

void clearBrainScr()
{
    Brain.Screen.clearScreen();
}

void clearControllerScr()
{
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("                                                         ");        
}