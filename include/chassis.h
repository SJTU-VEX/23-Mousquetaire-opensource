#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "PID.h"
#include "vex.h"
#include "geometry.h"
#include "parameters.h"
#include <math.h>
#include "calc.h"
#include "position.h"


class Chassis
{
private:
    /*----------------------------------------------------------------------------*/
    /*    为了融合手动与自动控制                                                   */ 
    /*    将底盘速度分为手动控制部分与自动控制部分                                  */ 
    /*    二者相加后为底盘控制速度                                                 */
    /*----------------------------------------------------------------------------*/

    //机器人坐标系速度，机器朝向为y轴，机器人朝向的右边为x轴
    Vector robotVel;

    //机器人角速度
    double robotVelR;

    //机器人坐标系下手动控制的速度
    Vector manualVel;
    double manualVelR;
    //机器人坐标系下自动控制的速度
    Vector autoVel;
    double autoVelR;

    double wheelVel[4] = {0};
    double wheelVolt[4] = {0};

    vex::brakeType stopBrakeType;

    // 轮子正方向为逆时针，以下角度为轮子正方向与x轴正方向的夹角，分别对应为
    // 右前 左前 左后 右后
    double wheelTheta[4] = {M_PI * 3 / 4, -M_PI * 3 / 4, -M_PI / 4, M_PI / 4};

    double speedGain = 127;

    Vector calcRobotVel(Vector Vel);
    void calcWheelVel();
    void calcWheelVolt();
    void setMotorVolt();

    // double updateRobotHeading();

public:
    static Chassis *getInstance(){
        static Chassis *c = NULL;
        if (c == NULL){
            c = new Chassis();
        }
        return c;
    }
    static void deleteInstance(){
        Chassis *c = Chassis::getInstance();
        if(c != NULL){
            delete c;
            c = NULL;
        }
    }
    Chassis();
    void setStopBrakeType(brakeType brake);
    void manualSetWorldVel(Vector Vel, double VelR);
    void manualSetRobotVel(Vector Vel, double VelR);
    void autoSetWorldVel(Vector Vel, double VelR);
    void autoSetRobotVel(Vector Vel, double VelR);
    void chassisBrake(brakeType brake);
    void chassisRun();
};

void updateChassis();


#endif