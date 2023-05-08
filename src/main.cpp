/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */ 
/*    Author:       SJTU VEX                                                  */
/*    Created:      Wed Dec 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "vex.h"
#include "basic-functions.h"
#include "auto-functions.h"
#include "usercontrol.h"
#include "position.h"
#include "autonomous.h"
#include "flyWheel.h"
#include "chassis.h"
#include "debugger.h"
#include "autoAiming.h"
#include "controller.h"
#include "distance.h"

#include <stdlib.h>
#include <iostream>
#include "adjusment.h"
using namespace std;
using namespace vex;

#ifdef COMPETITION
competition Competition;
#endif

// 定义Competiton会奇妙进入手动程序

int main()
{
    // vexcodeInit();
    cout << "start" << endl;
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    cout << "calibrated!" << endl;
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("         calibrated!");
    Position *p = Position::getInstance();
    setAimingStatus(false);
    
    setFlyWheelSpeed(0);
    thread FlyWheelControl(updateFlyWheel);
    thread UpdatePos(updatePosition);
    thread UpdateChassis(updateChassis);
    thread AutonAiming(autonAiming);
    thread AutonAimingVision(autonAiming_vision);
    thread TController(defineController);
    thread DistanceControl(updateDistance);

    // thread TestDistance(updateDistance);
    
    // Position::getInstance()->setGlobalPosition(39.034, 18.598);
    // Position::getInstance()->setGlobalPosition(3, 0);

    #ifdef COMPETITION_RIGHT
    Position::getInstance()->setGlobalPosition(25.707, 93.893);
    setIMUHeading(90);
    #endif

    #ifdef COMPETITION_RIGHT_Oliver
    Position::getInstance()->setGlobalPosition(25.707, 93.893);
    setIMUHeading(90);
    #endif

    #ifdef FIFTEEN
    Position::getInstance()->setGlobalPosition(25.707, 93.893);
    setIMUHeading(90);
    #endif

    #ifdef SKILL
    Position::getInstance()->setGlobalPosition(102.853, 22.783);
    #endif

    #ifdef COMPETITION
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    #endif
    
    #ifdef MANUAL
    usercontrol();
    #endif
    // moveToWithHeading(0, 100, 0, 100);
    
    #ifdef debug
    debugControl();
    #endif

    while (true){

        #ifdef AUTO
        if(press_B){
            autonomous();
            press_B = false;
        }
        #endif
        
        Point pos = Position::getInstance()->getPos();
        // Brain.Screen.clearScreen();
        Brain.Screen.setCursor(2, 2);
        Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
        // cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
        this_thread::sleep_for(10);
    }
}
