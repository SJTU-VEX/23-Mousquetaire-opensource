#include "basic-functions.h"
#include "PID.h"
#include "parameters.h"
#include "chassis.h"
#include "position.h"
#include "auto-functions.h"
#include "autoAiming.h"
#include <iostream>
using namespace std;

bool isAiming = false;
bool isAiming_vision = false;
int offset = 0;

int autonAiming_vision()
{
    float output = 0;
    auto pid = PID();
    pid.setTarget(CENTER_FOV + offset);
    pid.setCoefficient(0.35, 0, 0.85);
    
    // pid.setCoefficient(2, 0, 0);
    pid.setIMax(25);
    pid.setIRange(10);
    pid.setErrorTolerance(5);
    pid.setDTolerance(5);
    pid.setJumpTime(100);
    MyTimer adjustTimer;
    bool adjustFlag = false;
    double head1 = 0, head2 = 0;
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    while (true)
    {
        pid.setTarget(CENTER_FOV + offset);
        VisionSensor.takeSnapshot(VisionSensor__SIG_TAR);
        pid.update(VisionSensor.largestObject.centerX);
        output = pid.getOutput();
        if (isAiming_vision)
        {
            // if ((Point(300.777, 25.0472) - Position::getInstance()->getPos()).mod() <= 180)
            Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), -output);
            if (abs(VisionSensor.largestObject.centerX - CENTER_FOV) < 2){
                if (!adjustFlag && adjustTimer.getTime() > 1000){
                    adjustTimer.reset();
                    adjustFlag = true;
                    head1 = IMUHeading() + 180;
                    while (head1 > 180) head1 -= 360;
                    while (head1 < -180) head1 += 360;
                    Point pos = Position::getInstance()->getPos();
                    x1 = pos._x;
                    y1 = pos._y;
                }
                else if (adjustTimer.getTime() > 1000 && adjustFlag){
                    head2 = IMUHeading() + 180;
                    while (head2 > 180) head2 -= 360;
                    while (head2 < -180) head2 += 360;
                    Point pos = Position::getInstance()->getPos();
                    x2 = pos._x;
                    y2 = pos._y;
                    double dx = x2 - x1, dy = y2 - y1;
                    if (Vector(dx, dy).mod() < 1){
                        adjustFlag = false;
                        adjustTimer.reset();
                        continue;
                    }
                    else{
                        double d_theta = abs(head2 - head1); // 两瞄准方向夹角
                        // while (d_theta > 180) d_theta -= 360;
                        // while (d_theta < -180) d_theta += 360;
                        // d_theta = abs(d_theta);
                        double head3 = (90 - Vector(dx, dy).dir());
                        double d_phi = abs(head3 - head1); // 1号位置所在顶点三角形内角
                        double d = Vector(dx, dy).mod() / sin(d_theta) * sin(d_phi); //正弦定理求长度
                        double r = deg2rad(90 - head2); // 2号位置相对于high goal位置，极坐标角度
                        double real_x = targetPosX + d * cos(r);
                        double real_y = targetPosY + d * sin(r);

                        Position::getInstance()->setGlobalPosition(real_x, real_y);
                        Controller.rumble("-");
                        adjustFlag = false;
                        adjustTimer.reset();
                        continue;
                    }
                }
                
            }
            // else
                // aimAt(300.777, 25.0472);
            // cout << "isAiming" << endl;
            // cout << "pid: " << output << endl;
        }
        this_thread::sleep_for(20);
    }
    return 1;
}

int autonAiming()
{
    Point _tarPos = Point(targetPosX, targetPosY);
    Point pos = Position::getInstance()->getPos();
    double _tarAng = 90 - (_tarPos - pos).dir();
    _tarAng += 180;
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;

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

    while (true)
    {
        double speedX = Position::getInstance()->getXSpeed();
        double speedY = Position::getInstance()->getYSpeed();
        double dist = (Position::getInstance()->getPos() - Point(targetPosX, targetPosY)).mod();
        double discSpeed = 420; // cm * s ^ -1
        // 出射速度 460 ~ 480 cm/s
        double t = dist / discSpeed; /*????*/

        _tarPos = Point(targetPosX - speedX * t, targetPosY - speedY * t);
        pos = Position::getInstance()->getPos();
        _tarAng = 90 - (_tarPos - pos).dir();
        _tarAng += 180;
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
        double output = pid.getOutput(); 
        if(abs(output) >= 60) output = 60 * sign(output);     

        if (isAiming)
        {
            // aimPreciselyAt(targetPosX - speedX * t, targetPosY - speedY * t);
            // aimPreciselyAt(targetPosX, targetPosY);
            Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), output);
        }
        this_thread::sleep_for(20);
    }
    return 1;
}

void setAimingStatus(bool _input)
{
    isAiming = _input;
    offset = 0;
}

void setVisionAimingStatus(bool _input)
{
    isAiming_vision = _input;
    offset = 0;
}

void setAimingStatus(bool _input, int _offset)
{
    isAiming = _input;
    offset = _offset;
}

void setVisionAimingStatus(bool _input, int _offset)
{
    isAiming_vision = _input;
    offset = _offset;
}