#include "PID.h"
#include "parameters.h"
#include "robot-config.h"
#include "basic-functions.h"
#include "chassis.h"
#include "position.h"
#include "trajectory.h"
// #include "controller.h"
#include "auto-functions.h"
#include <iostream>
using namespace std;

void moveToWithHeading(double _x, double _y, double _tarAng, double _maxSpeed)
{
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double dis = (_tarPos - pos).mod();
    double time = 1000 * dis / 100;
    if (time < 2000) time = 2000;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getPerciseTraj(_tarPos, _tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= time)
    {
        traj->update();
        // cout << Position::getInstance()->getPos()._x << " " << Position::getInstance()->getPos()._y << " " << IMUHeading() << endl;
        // cout << "Vel: " << traj->getVelocity()._x << " " << traj->getVelocity()._y << " " << traj->getVelocityR() << endl;
        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    delete traj;
    this_thread::sleep_for(50);
}

void quickMoveToWithoutStop(double _x, double _y, double _tarAng, double _maxSpeed){
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double dis = (_tarPos - pos).mod();
    double time = 1000 * dis / 100;
    if (time < 2000) time = 2000;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getQuickTrajWithoutStop(_tarPos, _tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= time)
    {
        traj->update();
        // cout << Position::getInstance()->getPos()._x << " " << Position::getInstance()->getPos()._y << " " << IMUHeading() << endl;
        // cout << "Vel: " << traj->getVelocity()._x << " " << traj->getVelocity()._y << " " << traj->getVelocityR() << endl;
        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    this_thread::sleep_for(100);
}

void quickMoveToWithHeading(double _x, double _y, double _tarAng, double _maxSpeed){
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double dis = (_tarPos - pos).mod();
    double time = 1000 * dis / 100;
    if (time < 1500) time = 1500;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getQuickTraj(_tarPos, _tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= time+500)
    {
        traj->update();
        // cout << Position::getInstance()->getPos()._x << " " << Position::getInstance()->getPos()._y << " " << IMUHeading() << endl;
        // cout << "Vel: " << traj->getVelocity()._x << " " << traj->getVelocity()._y << " " << traj->getVelocityR() << endl;
        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    this_thread::sleep_for(30);
}

void aimAt(double _x, double _y, double _offset)
{
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double _tarAng = 90 - (_tarPos - pos).dir() + _offset;
    _tarAng += 180;
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getPerciseTurnTraj(_tarAng, 60);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= 1000)
    {
        traj->update();
        // cout << Position::getInstance()->getPos()._x << " " << Position::getInstance()->getPos()._y << " " << IMUHeading() << endl;
        // cout << "Vel: " << traj->getVelocity()._x << " " << traj->getVelocity()._y << " " << traj->getVelocityR() << endl;
        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    delete traj;
    this_thread::sleep_for(100);
}

void aimPreciselyAt(double _x, double _y, double _offset)
{
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double _tarAng = 90 - (_tarPos - pos).dir() + _offset;
    _tarAng += 180;
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;

    float output = 0;
    auto pid = PID();
    pid.setTarget(_tarAng);
    pid.setCoefficient(2, 0.2, 8);
    // pid.setCoefficient(2, 0, 0);
    pid.setIMax(40);
    pid.setIRange(10);
    pid.setErrorTolerance(2);
    pid.setDTolerance(5);
    pid.setJumpTime(100);
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

void turnTo(double _tarAng, double _maxSpeed)
{
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getPerciseTurnTraj(_tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= 1300)
    {
        traj->update();
        
        #ifdef debug
        cout << traj->getVelocityR() << ";" << IMUHeading() << endl;
        #endif

        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    delete traj;
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    this_thread::sleep_for(100);
}

void quickTurnTo(double _tarAng, double _maxSpeed)
{
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;
    Trajectory *traj = TrajectoryFactory::getQuickTurnTraj(_tarAng, _maxSpeed);
    MyTimer mytimer;
    while (!traj->targetArrived() && mytimer.getTime() <= 1300)
    {
        traj->update();
        
        #ifdef debug
        cout << traj->getVelocityR() << ";" << IMUHeading() << endl;
        #endif

        Chassis::getInstance()->autoSetWorldVel(traj->getVelocity(), traj->getVelocityR());
        this_thread::sleep_for(50);
    }
    delete traj;
    Chassis::getInstance()->chassisBrake(vex::brakeType::hold);
    this_thread::sleep_for(100);
}

void timerMove(Vector vel, int msec)
{
    Chassis::getInstance()->autoSetRobotVel(vel, 0);
    this_thread::sleep_for(msec);
    Chassis::getInstance()->chassisBrake(brakeType::brake);
}

void shoot(int _num, int _msec)
{
    for (int i = 1; i <= _num; ++i)
    {
        Piston_Trigger.set(true);
        this_thread::sleep_for(50);
        Piston_Trigger.set(false);
        this_thread::sleep_for(150 + _msec);
    }
    #ifdef FourBarProcess
    if (_num > 2)
    {
        Piston_Trigger.set(true);
        this_thread::sleep_for(50);
        Piston_Trigger.set(false);
    }
    #endif
}

/**
 * @brief 定时前进，朝向为机器坐标系朝�?
 * 
 * @param _power 
 * @param _duration 
 * @param _targetHeading 
 */
void timerForwardWithHeading(float _power, int _duration, double _targetHeading)
{
    auto myTimer = MyTimer();
    double startHeading = IMUHeading();
    _targetHeading += startHeading;
    while (myTimer.getTime() < _duration)
    {
        double headingError = _targetHeading - IMUHeading();
        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;
        double powerTurn = headingError * 2.0; // kp = 2.0
        if (fabs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(Vector(0, _power), powerTurn);
    }
    Chassis::getInstance()->chassisBrake(brakeType::brake);
}

void turnRoller(){
    moveIntaker(0);
    timerForwardWithHeading(50, 400, 0);
    Chassis::getInstance()->chassisBrake(brakeType::hold);
    Motor_Intaker1.resetPosition();
    moveIntaker(100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) >= 130);
    moveIntaker(0);
    Chassis::getInstance()->chassisBrake(brakeType::coast);
    timerForwardWithHeading(-30, 250, 0);
}

void intake3DiscsWithPiston(double _x, double _y, double _ang)
{
    Piston_IntakerLifter.set(true);
    moveIntaker(0);
    moveToWithHeading(_x, _y, _ang, 70);
    timerForwardWithHeading(40, 300, 0);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(300);
    timerForwardWithHeading(-40, 500, 0);
    moveIntaker(100);
    moveToWithHeading(_x, _y, _ang, 70);
    this_thread::sleep_for(500);
    Piston_IntakerLifter.set(true);
    timerForwardWithHeading(45, 300, 0);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(2000);
    // moveIntaker(0);
}


void intake3DiscsWithoutPiston(double _x, double _y, double _ang)
{ 
    moveIntaker(100);
    moveToWithHeading(_x, _y, _ang, 60);
    timerForwardWithHeading(18, 3000, 0);
}



