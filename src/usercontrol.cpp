#include "vex.h"
#include "basic-functions.h"
#include "controller.h"
#include "adjusment.h"
#include "parameters.h"
#include "flyWheel.h"
#include "position.h"
#include "chassis.h"
#include "usercontrol.h"
#include "autoAiming.h"
#include "autonomous.h"
#include <iostream>
#include <mutex>
using namespace std;
using namespace vex;

bool isUsrCtl = true, last_isUsrCtl = true;
mutex m;

/**
 * @brief 底盘车头控制
 * 
 */
void baseControl()
{
    double a1, a3, a4;
    a1 = (abs(A1) < 3) ? 0 : A1;
    a3 = (abs(A3) < 10) ? 0 : A3;
    a4 = (abs(A4) < 10) ? 0 : A4;
    Vector v = Vector(a4, a3);
    if (v.mod() > 100) v = v / v.mod() * 100;

    //wall shooting 滤除旋转抖动
    if (abs(A2) > 75)
    {
        a1 = 0;
    }

    Chassis::getInstance()->manualSetRobotVel(v, a1);
}

/**
 * @brief 底盘人头控制
 * 
 */
void baseControlbyHeading()
{
    double a1, a3, a4;
    a1 = (abs(A1) < 3) ? 0 : A1;
    a3 = (abs(A3) < 10) ? 0 : A3;
    a4 = (abs(A4) < 10) ? 0 : A4;
    
    #ifdef COMPETITION_RIGHT
    Vector v = Vector(-a4, -a3);    // 取决于操作手初始站位
    #endif

    #ifdef COMPETITION_RIGHT_Oliver
    Vector v = Vector(-a4, -a3);    // 取决于操作手初始站位
    #endif

    #ifdef SKILL
    Vector v = Vector(a4, a3);    // 取决于操作手初始站位
    #endif

    #ifdef debug
    Vector v = Vector(a4, a3);    // 取决于操作手初始站位
    #endif

    if (v.mod() > 100) v = v / v.mod() * 100;

    //wall shooting 滤除旋转抖动
    if (abs(A2) > 75)
    {
        a1 = 0;
    }
    
    Chassis::getInstance()->manualSetWorldVel(v, a1);
}

void intakerControl()
{
    // moveIntaker((R1 - RIGHT) * 100);
    if(R1 && !L1) moveIntaker(100);//R1控制intaker反转, 吃盘子
    else if (L1 && !R1) moveIntaker(-100);//L1控制intaker正转, 滚roller
    else moveIntaker(0);
}

void intakerLift()
{
    if (Y)
    {
        Piston_IntakerLifter.set(true);
    }
    else
    {
        Piston_IntakerLifter.set(false);
    }
}

void anglerControl()
{
    if (L2)
    {
        Piston_Angler.set(true);
    }
    else
    {
        Piston_Angler.set(false);
    }
}

void triggerControl()
{
    if (R2)
    {
        Piston_Trigger.set(true);
        // Controller.rumble(".");
        delay(50);
        Piston_Trigger.set(false);
        delay(119);
    }
}

void flyWheelControl(){
    if(X && !A && !B)
    {
        setFlyWheelSpeed(465);
        // setFlyWheelSpeed(408);
        // setFlyWheelSpeed(390);
    }
    if(A && !X && !B)
    {
        setFlyWheelSpeed(365);
        // setFlyWheelSpeed(335);
        // setFlyWheelSpeed(320);
    }
    if(B && !A && !X)
    {
        setFlyWheelSpeed(0);
    }
}

void deployerControl(){
    if (UP && LEFT){
        Piston_Deployer.set(true);
    }
    else{
        Piston_Deployer.set(false);
    }
}

void positionAiming(){
    static MyTimer timerTrigger;
    static bool flag = false, lastFlag = false;
    if (A2 > -80)
    {
       timerTrigger.reset();
       flag = false;
    }
    else
    {
        if (!flag && timerTrigger.getTime() > 200)
        {
            flag = true;
        }
    }

    if (flag)
    {
        setAimingStatus(true);
    }
    if (lastFlag && !flag)
    {
        setAimingStatus(false);
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlag = flag;
}

void head45Degree(){
    static MyTimer timerTrigger;
    static bool flag = false, lastFlag = false;
    if (A2 > -80)
    {
       timerTrigger.reset();
       flag = false;
    }
    else
    {
        if (!flag && timerTrigger.getTime() > 500)
        {
            flag = true;
        }
    }

    if (flag)
    {
        // turnToLock(135);
        turnToLock(45);
    }
    if (lastFlag && !flag)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlag = flag;
}

void WallShoot()
{
    static MyTimer timerL, timerR;
    static bool flagL = false, lastFlagL = false;
    static bool flagR = false, lastFlagR = false;
    static float LAngel = 14, RAngel = 259;
    if (A2 < 85)
    {
       timerR.reset();
       flagR = false;
    }
    else
    {
        if (!flagR && timerR.getTime() > 50)
        {
            flagR = true;
        }
    }

    if (A2 > -85)
    {
       timerL.reset();
       flagL = false;
    }
    else
    {
        if (!flagL && timerL.getTime() > 50)
        {
            flagL = true;
        }
    }

    if (flagL)
    {
        if (L2)
        {
            turnToLock(LAngel+1);
        }
        else
        {
            turnToLock(LAngel);
        }
    }
    if (lastFlagL && !flagL)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    if (flagR)
    {
        if (L2)
        {
            turnToLock(RAngel-3);
        }
        else
        {
            turnToLock(RAngel);
        }
    }
    if (lastFlagR && !flagR)
    {
        Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    }

    lastFlagL = flagL;
    lastFlagR = flagR;

}

void usrCtlThread(void * childThread)
{
    vex::thread* Thread = (vex::thread*)childThread;
    vex::thread* T = NULL;
    while (true)
    {
        // if(isUsrCtl && !last_isUsrCtl)
        // {
        //     //重置所有外设

        //     //中断进程
        //     Thread->interrupt();
        //     Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
        // }

        /*---------技能赛手动自动切换 ———— 手动 -> 自动：按LEFT键 / 自动 -> 手动：按X&&A键--------------*/
       
        if (press_DOWN && T == NULL){ //进入自动
            isUsrCtl = !isUsrCtl;
            press_DOWN = false;
            // cout << "LEFT Pressed" << endl;
            T = new thread(autonomous);
            // cout << "111" << endl;
            
        }
        if(X && A  && T != NULL){ //中断自动进手动
            isUsrCtl = !isUsrCtl;
            T->interrupt();
            delete T;
            T = NULL;
            Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
        }      

        /*---------------------------------------------------------------------------------------*/


        if(isUsrCtl)
        {
            static bool brakeType = false;
            //底盘控制
            // baseControl();
            baseControlbyHeading();

            //其余组件控制
            intakerControl();
            anglerControl();
            triggerControl();
            intakerLift();
            flyWheelControl();
            deployerControl();
            WallShoot();       //靠墙射击
            // head45Degree(); //lock heading 
            // positionAiming();

            // 运行自动程序:
            // if(X && A && !UP && !LEFT){
            //     autonomous();
            // }

            // //自瞄开关
            // if(release_L2)
            // {
            //     setVisionAimingStatus(false);
            //     Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
            //     release_L2 = false;
            // }
            // else if(press_L2)
            // {
            //     setVisionAimingStatus(true);
            //     press_L2 = false;
            // }
            
            //底盘锁定模式切换
            // if (press_DOWN) {
            //     brakeType = !brakeType;
            //     if (brakeType){
            //         Chassis::getInstance()->chassisBrake(brakeType::hold);
            //         Chassis::getInstance()->setStopBrakeType(brakeType::hold);
            //     }
            //     else{
            //         Chassis::getInstance()->chassisBrake(brakeType::coast);
            //         Chassis::getInstance()->setStopBrakeType(brakeType::coast);
            //     }
            //     press_DOWN = false;
            // }
        }
        this_thread::sleep_for(RefreshTime);
    }
}

void autoCtlThread()
{
    //单状态机实现，状态变量为全局变量，每一个状态完成一个非阻塞自动函数
    // while(isUsrCtl){
    //     cout << "isUsrCtl" << endl;
    //     this_thread::sleep_for(1000);
    // }
    // cout << "auto" << endl;
    // autonomous();
    return;
}

void usercontrol()
{
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
    setFlyWheelSpeed(0);
    setAimingStatus(false);
    cout << "enter user control" << endl;

    thread AutoCtl(autoCtlThread);
    // thread UsrCtl(usrCtlThread, &AutoCtl);
    usrCtlThread(&AutoCtl);
    // UsrCtl.join();
    // while (true)
    // {
    //     this_thread::sleep_for(10);
    //     // Controller.Screen.setCursor(5, 1);
    //     // Controller.Screen.print(isUsrCtl? "USR" : "AUTO");
    // }
}
