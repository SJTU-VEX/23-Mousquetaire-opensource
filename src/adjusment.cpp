#include "position.h"
#include "auto-functions.h"
#include "chassis.h"
#include "basic-functions.h"
#include "adjusment.h"
#include <iostream>
using namespace std;

/**
 * @brief 飞轮自动打点
 * 
 */
void testFlyWheel(){
    for (int i = 25; i <= 50; ++i){
        Motor_FlyWheel1.spin(directionType::fwd, 2 * i, pct);
        Motor_FlyWheel2.spin(directionType::fwd, 2 * i, pct);
        this_thread::sleep_for(6000);
        double spd = (Motor_FlyWheel1.velocity(velocityUnits::rpm) + Motor_FlyWheel2.velocity(velocityUnits::rpm)) / 2;
        cout << 2 * i << " " << spd << endl;
    }
}

void turnPerciselyTo(double _tarAng){

    double d = _tarAng - IMUHeading();
    while (d > 180){
        _tarAng -= 360;
        d -= 360;
    }
    while (d < -180){
        _tarAng += 360;
        d += 360;
    }

    float output = 0;
    auto pid = PID();
    pid.setTarget(_tarAng);
    pid.setCoefficient(3, 0.2, 6);
    // pid.setCoefficient(2, 0, 0);
    pid.setIMax(50);
    pid.setIRange(10);
    pid.setErrorTolerance(2);
    pid.setDTolerance(5);
    pid.setJumpTime(200);
    MyTimer mytimer;
    while (!pid.targetArrived() && mytimer.getTime() <= 1500)
    {
        double d = _tarAng - IMUHeading();
        while (d < -180){
            d += 360;
            _tarAng += 360;
        }
        while (d > 180){
            d -= 360;
            _tarAng -= 360;
        }
        pid.setTarget(_tarAng);
        pid.update(IMUHeading());
        output = pid.getOutput(); 
        if(abs(output) >= 60) output = 60 * sign(output);
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), output);
        this_thread::sleep_for(20);
    }
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    this_thread::sleep_for(300);
}

void turnToLock(double _tarAng){
    float output = 0;
    static auto pid = PID();

    double curAng = IMUHeading();
    double d = _tarAng - curAng;
    while (d < -180){
        d += 360;
        _tarAng += 360;
    }
    while (d > 180){
        d -= 360;
        _tarAng -= 360;
    }

    pid.setCoefficient(1.5,0.1,6);
    pid.setIRange(10);
    pid.setIMax(25);
    pid.setErrorTolerance(1);
    

    pid.setTarget(_tarAng);
    pid.update(curAng);
    output = pid.getOutput(); 
    // cout << "Error: " << d << endl;
    if(abs(output) >= 100) output = 100 * sign(output);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), output);
}

void spinCW(){
    moveClockwise(-50);
    this_thread::sleep_for(6200);
    moveClockwise(0);
    Position::getInstance()->setGlobalPosition(0, 0);
    turnPerciselyTo(0);
}

void spinCC(){
    moveClockwise(50);
    this_thread::sleep_for(6200);
    moveClockwise(0);
    Position::getInstance()->setGlobalPosition(0, 0);
    turnPerciselyTo(0);
}

void autoSpinRadiusCW(){
    double SpinRadiusSumL = 0, SpinRadiusSumR = 0;
    for (int i = 1; i <= 5; ++i){
        EncoderL.setPosition(0, rotationUnits::deg);
        EncoderR.setPosition(0, rotationUnits::deg);
        spinCW();
        SpinRadiusSumL += Position::getInstance()->getLMileage() / (8 * M_PI);
        SpinRadiusSumR += Position::getInstance()->getRMileage() / (8 * M_PI);
    }
    SpinRadiusSumL /= 5;
    SpinRadiusSumR /= 5;
    cout << "SpinRadiusLCW: " << SpinRadiusSumL << endl;
    cout << "SpinRadiusRCW: " << SpinRadiusSumR << endl;
}

void autoSpinRadiusCC(){
    double SpinRadiusSumL = 0, SpinRadiusSumR = 0;
    for (int i = 1; i <= 5; ++i){
        EncoderL.setPosition(0, rotationUnits::deg);
        EncoderR.setPosition(0, rotationUnits::deg);
        spinCC();
        SpinRadiusSumL += Position::getInstance()->getLMileage() / (8 * M_PI);
        SpinRadiusSumR += Position::getInstance()->getRMileage() / (8 * M_PI);
    }
    SpinRadiusSumL /= 5;
    SpinRadiusSumR /= 5;
    cout << "SpinRadiusLCC: " << SpinRadiusSumL << endl;
    cout << "SpinRadiusRCC: " << SpinRadiusSumR << endl;
}

void adjustForward(float _power){
    double head = IMUHeading();
    double r = 0;
    if (head < 40) r = -5;
    if (head > 300) r = 5;
    Chassis::getInstance()->autoSetWorldVel(Vector(0, _power), r);
}

float adjustGetPos(){
    return (float)Position::getInstance()->getPos()._y;
}