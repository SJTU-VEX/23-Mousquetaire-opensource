#include "vex.h"
#include "chassis.h"
#include "basic-functions.h"
#include "position.h"
#include <cmath>
#include <iostream>
using namespace vex;
using namespace std;

/// @brief 底盘构造函数
Chassis::Chassis()
{
    robotVel.resetV();
    manualVel.resetV();
    autoVel.resetV();
    stopBrakeType = coast;
    robotVelR = 0;
    manualVelR = 0;
    autoVelR = 0;
}

/// @brief 【换算函数】将世界坐标系下的速度转换为机器人坐标系下的速度
Vector Chassis::calcRobotVel(Vector Vel)
{
    return Vel.rotateTrans(IMUHeading());
}

/// @brief 【换算函数】将机器人坐标系下的速度转换为每个轮子的速度
void Chassis::calcWheelVel()
{
    double tranSpeed[4] = {0}, rotaSpeed[4] = {0};
    double robotSpeed = robotVel.mod();
    double maxAbsVel = 0;
    double maxAbsWheelVel = 100;
    for (int i = 0; i < 4; i++)
    {
        tranSpeed[i] = robotVel * Vector(1, 0).rotateTrans(rad2deg(wheelTheta[i]));
        rotaSpeed[i] = -robotVelR;
        if (abs(tranSpeed[i]) > maxAbsVel) maxAbsVel = abs(tranSpeed[i]);  
    }
    for (int i = 0; i < 4; i++)
    {
        tranSpeed[i] = (maxAbsVel == 0) ? 0 : tranSpeed[i] / maxAbsVel * robotSpeed; //将轮子平移速度按robotSpeed比例放大
        wheelVel[i] = tranSpeed[i] + rotaSpeed[i];
        if (abs(wheelVel[i]) > maxAbsWheelVel) maxAbsWheelVel = abs(wheelVel[i]);
        // cout << "rota" << i <<": " <<rotaSpeed[i] << endl;
    }

    //将车轮速度归一到（-100 ~ 100）
    for (int i = 0; i < 4; i++)
    {
        wheelVel[i] = wheelVel[i] / maxAbsWheelVel * 100;
        // cout << "wheel" << i << ": " << wheelVel[i] << endl;
    }
}

/// @brief 【换算函数】将轮速转换为对应电机电压
void Chassis::calcWheelVolt()
{
    for (int i = 0; i < 4; i++)
    {
        wheelVolt[i] = wheelVel[i] * speedGain;
        if (abs(wheelVolt[i]) < 1000)//1500
            wheelVolt[i] = 0;
    }
}

/// @brief 按指定电压驱动电机
void Chassis::setMotorVolt()
{
    if (wheelVolt[0] == 0)
    {
        Motor_BaseRFU.stop(stopBrakeType);
        Motor_BaseRFD.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseRFU.spin(directionType::fwd, wheelVolt[0], voltageUnits::mV);
        Motor_BaseRFD.spin(directionType::fwd, wheelVolt[0], voltageUnits::mV);
    }

    if (wheelVolt[1] == 0)
    {
        Motor_BaseLFU.stop(stopBrakeType);
        Motor_BaseLFD.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseLFU.spin(directionType::fwd, wheelVolt[1], voltageUnits::mV);
        Motor_BaseLFD.spin(directionType::fwd, wheelVolt[1], voltageUnits::mV);
    }

    if (wheelVolt[2] == 0)
    {
        Motor_BaseLBU.stop(stopBrakeType);
        Motor_BaseLBD.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseLBU.spin(directionType::fwd, wheelVolt[2], voltageUnits::mV);
        Motor_BaseLBD.spin(directionType::fwd, wheelVolt[2], voltageUnits::mV);
    }

    if (wheelVolt[3] == 0)
    {
        Motor_BaseRBU.stop(stopBrakeType);
        Motor_BaseRBD.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseRBU.spin(directionType::fwd, wheelVolt[3], voltageUnits::mV);
        Motor_BaseRBD.spin(directionType::fwd, wheelVolt[3], voltageUnits::mV);
    }
}

/// @brief 强制底盘刹车（注意会收到电机驱动线程的影响，在使用该函数时确保不与电机驱动线程冲突）
/// @param brake 底盘刹车类型（coast，brake，hold）
void Chassis::chassisBrake(brakeType brake)
{
    wheelVolt[0] = 0;
    wheelVolt[1] = 0;
    wheelVolt[2] = 0;
    wheelVolt[3] = 0;
    
    Motor_BaseRFU.stop(brakeType::brake);
    Motor_BaseRFD.stop(brakeType::brake);

    Motor_BaseLFU.stop(brakeType::brake);
    Motor_BaseLFD.stop(brakeType::brake);

    Motor_BaseLBU.stop(brakeType::brake);
    Motor_BaseLBD.stop(brakeType::brake);

    Motor_BaseRBU.stop(brakeType::brake);
    Motor_BaseRBD.stop(brakeType::brake);
}

/// @brief 底盘电机驱动函数，将作为单独线程不断刷新
void Chassis::chassisRun()
{ 
    setMotorVolt();
}

/// @brief 【手动控制】通过世界坐标系速度驱动底盘
/// @param Vel 世界坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::manualSetWorldVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    manualVel = calcRobotVel(Vel);
    manualVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【手动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::manualSetRobotVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    manualVel = Vel;
    manualVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【自动控制】通过世界坐标系速度驱动底盘
/// @param Vel 世界坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetWorldVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = calcRobotVel(Vel);
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【自动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度向量，速度向量大小（模）在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetRobotVel(Vector Vel, double VelR)
{
    if (Vel.mod() > 100){
        Vel = Vel / Vel.mod() * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = Vel;
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (robotVel.mod() > 100){
        robotVel = robotVel / robotVel.mod() * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}


/**
 * @brief 设置停止时的刹车类型
 * 
 * @param brake 
 */
void Chassis::setStopBrakeType(brakeType brake){
    stopBrakeType = brake;
}

void updateChassis()
{
    while(true)
    {
        Chassis::getInstance()->chassisRun();
        this_thread::sleep_for(RefreshTime);
    }   
}

