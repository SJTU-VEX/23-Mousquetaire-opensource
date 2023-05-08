#include "autonomous.h"
#include "auto-functions.h"
#include "chassis.h"
#include "position.h"
#include "basic-functions.h"
#include "flyWheel.h"
#include "vex.h"
#include <iostream>
using namespace std;
using namespace vex;

#define SHoot

void testRoller(){
    turnRoller();
    // cout << "ending autonomous" << endl;
    // ChassisControl.interrupt();
    while(true){
        Position *p = Position::getInstance();
        // cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
        this_thread::sleep_for(100);
    }
}

/**
 * @brief 技能赛自动
 * 
 */
#ifdef SKILL
void autonomous(){
    cout << "enter auto functions" << endl;
    // Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();

    MyTimer timer;
    timer.reset();

    // 前进到导入台，清空

    /*------------------------------------------------------------------------------------------*/

    setFlyWheelSpeed(368);
    aimAt(283, 28, 3.5);
    this_thread::sleep_for(1200); //等待飞轮加速，时间可优化
    #ifdef SHoot
    moveIntaker(80);
    shoot(3,200 + 150);
    shoot(8,400 + 150);                //防止四盘，存在多余射击次数，可以优化
    #endif

    /*------------------------------------------------------------------------------------------*/

    // 吃三连
    timerMove(Vector(50, 0), 300);
    quickMoveToWithHeading(200, 48, 50, 100);
    quickMoveToWithHeading(200, 98, 49, 60 + 30);

    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        Controller.rumble("-");
        fourBarFlag = false;
    }
    #endif

    setFlyWheelSpeed(364);
    // quickMoveToWithHeading(192 + 16, 94.2126 + 5, 45, 60); // 向外平移，防止撞场地
    quickMoveToWithHeading(192 + 10, 94.2126 + 5, -50, 60+30);
    

    aimAt(283, 28, -1);

    #ifdef SHoot
    shoot(3,300);
    #endif

    /*------------------------------------------------------------------------------------------*/

    // 吃三斜
    moveIntaker(70);
    quickMoveToWithHeading(179, 104, 43, 100);

    // timerForwardWithHeading(60, 1800, 0);
    quickMoveToWithHeading(179 + 112 + 5, 104 + 112, 44, 60); //三斜吃盘不稳定可以考虑调参
    // moveToWithHeading(237.324 + 20, 177.755 + 20, 45, 100);

    setFlyWheelSpeed(372);

    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        Controller.rumble("-");
        fourBarFlag = false;
    }
    #endif

    quickMoveToWithHeading(250.346-3, 140 + 15+3, 0, 100);

    aimAt(292, 25, -1 - 1 -1.5);

    #ifdef SHoot
    shoot(3,300);
    #endif

    /*------------------------------------------------------------------------------------------*/

    // 吃三竖
    // quickMoveToWithHeading(188.381, 154.0409, 100, 100);
    moveIntaker(80);
    quickMoveToWithHeading(216, 117, 135, 100);
    quickMoveToWithHeading(273 + 17 , 117, 135, 60);

    quickMoveToWithHeading(315, 154.0409, 11.467, 100);
    setFlyWheelSpeed(359);                                  //飞轮速度可优化        

    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        Controller.rumble("-");
        fourBarFlag = false;
    }
    #endif

    quickMoveToWithHeading(315, 94.0409 + 20, 11.467, 80);
    Piston_Angler.set(true);

    aimAt(290, 25, -1-1);

    #ifdef SHoot
    shoot(3,300);
    #endif

    Piston_Angler.set(false);

    /*------------------------------------------------------------------------------------------*/

    // shoot(3);

    // 吃三叠

    //以下两个操作防止撞场地    
    moveIntakerWithRPM(400);
    quickMoveToWithHeading(315, 120.0409 + 20 + 5, 11.467, 100);
    timerMove(Vector(-100, 0), 590);
    Piston_IntakerLifter.set(true);
    moveToWithHeading(140, 77, 225, 100);

    this_thread::sleep_for(200);    //确保车辆停稳，可以优化动作序列
    timerForwardWithHeading(100 , 180 , 0);
    this_thread::sleep_for(100);    //确保车辆停稳，可以优化动作序列

    Piston_IntakerLifter.set(false);

    // this_thread::sleep_for(500);    //确保车辆停稳，可以优化动作序列
    // timerForwardWithHeading(30 , 500 , 0);

    this_thread::sleep_for(1100);   //等待三叠吃盘，时间可优化
    
    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        Controller.rumble("-");
        fourBarFlag = false;
    }
    #endif

    setFlyWheelSpeed(359);
    timerMove(Vector(-100, 100), 430);  //防止撞场地
    moveToWithHeading(222.48 - 2 - 3, 6.1852-4, 264.925, 100);
    Piston_Angler.set(true);
    aimAt(290, 17, -1);

    #ifdef SHoot
    shoot(3,200);
    #endif

    /*------------------------------------------------------------------------------------------*/

    // 吃三叠
    moveIntakerWithRPM(400);
    Piston_IntakerLifter.set(true);
    moveToWithHeading(81.5+2, 60+2, 306, 100);
    // this_thread::sleep_for(800);    //确保车辆停稳，可以优化动作序列

    this_thread::sleep_for(100);    //确保车辆停稳，可以优化动作序列
    timerForwardWithHeading(50 , 180 , 0);
    // this_thread::sleep_for(500);

    Piston_IntakerLifter.set(false);

    // timerForwardWithHeading(50, 170 + 50, 0);
    // this_thread::sleep_for(100);    //确保车辆停稳，可以优化动作序列
    // timerForwardWithHeading(30 , 500 , 0);
    this_thread::sleep_for(500);

    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        Controller.rumble("-");
        fourBarFlag = false;
    }
    #endif

    setFlyWheelSpeed(359);
    quickMoveToWithHeading(120, 10 + 13, 264.925, 100);
    quickMoveToWithHeading(222.48 - 2 - 5, 6.1852-4, 264.925, 100);
    Piston_Angler.set(true);
    aimAt(290, 16, -1);

    #ifdef SHoot
    shoot(3,300);
    #endif

    /*------------------------------------------------------------------------------------------*/

    moveIntaker(70);
    quickMoveToWithHeading(16, 42+5, 260, 100);

    moveIntaker(0);
    
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    timerForwardWithHeading(50, 450, 0);    // 前移靠近Roller
    Chassis::getInstance()->chassisBrake(brakeType::hold);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), -20);
    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -160    -60 + 40);  // 转动Roller  //对面roller比较紧 要+60
    moveIntaker(1);
    this_thread::sleep_for(300);  //防止有的roller太松带着惯性滚 
    
    shoot();

    timerForwardWithHeading(-50, 200, 0);

    /*------------------------------------------------------------------------------------------*/

    moveIntaker(0);
    moveToWithHeading(44.6245, 9.74217, 190, 100);
    timerForwardWithHeading(60, 550, 0);    // 前移靠近Roller
    Chassis::getInstance()->chassisBrake(brakeType::hold);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), 20);
    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -160    - 60 +10);  // 转动Roller  //对面roller比较紧 要+60
    moveIntaker(1);
    this_thread::sleep_for(300);   //防止有的roller太松带着惯性滚 
    timerForwardWithHeading(-30, 250, 0);
    /*------------------------------------------------------------------------------------------*/
    // moveIntaker(0);
    // quickMoveToWithHeading(16, 42+5, 260, 100);
    // timerForwardWithHeading(50, 450, 0);    // 前移靠近Roller
    // Chassis::getInstance()->chassisBrake(brakeType::hold);
    // Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), -20);
    // Motor_Intaker1.resetPosition();
    // moveIntaker(-100);
    // waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -160    -60 + 40);  // 转动Roller  //对面roller比较紧 要+60
    // moveIntaker(1);
    // this_thread::sleep_for(300);  //防止有的roller太松带着惯性滚  
    // timerForwardWithHeading(-50, 200, 0);

    Chassis::getInstance()->setStopBrakeType(brakeType::coast);

    // this_thread::sleep_for(1100);

    /*------------------------------------------------------------------------------------------*/

    // 移动到指定位置释放Deploy 3 下

    moveToWithHeading(20, 15, 225, 100);

    while(timer.getTimeDouble() < 58)
    {
      this_thread::sleep_for(10);
    }

    Piston_Deployer.set(true);
    this_thread::sleep_for(200);
    Piston_Deployer.set(false);
    this_thread::sleep_for(100);
    Piston_Deployer.set(true);
    this_thread::sleep_for(200);
    Piston_Deployer.set(false);
    this_thread::sleep_for(100);
    Piston_Deployer.set(true);

    // Piston_IntakerLifter.set(true); ///////////////////////////////////

    // quickMoveToWithHeading(30, 45, 270, 100);
    // moveToWithHeading(30, 30, 225, 100);


    /*------------------------------------------------------------------------------------------*/
    // turnRoller();
    // // timerForwardWithHeading(50, 400, 0);
    // // moveIntaker(100);
    // // waitUntil(Motor_Intaker1.position(rotationUnits::deg) >= 130);
    // // moveIntaker(0);
    // timerForwardWithHeading(-40, 250, 0);

    

    // shoot(3);

    // setFlyWheelSpeed(0);
    // moveIntaker(0);
    
    /*
    setFlyWheelSpeed(315);
    moveIntaker(100);
    quickMoveToWithHeading(45, 0.9, 180, 100);
    this_thread::sleep_for(1000);
    moveIntaker(0);

    // 转动roller2
    // moveToWithHeading(44.6245 + 10, 9.74217, 180, 100);
    turnRoller();
    timerForwardWithHeading(-100, 300, 0);
    
    // 转动roller1
    moveToWithHeading(16, 44, 270, 100);

    // turn Roller
    moveIntaker(0);
    timerForwardWithHeading(50, 400, 0);
    Chassis::getInstance()->chassisBrake(brakeType::hold);
    Position::getInstance()->setGlobalPosition(0, Position::getInstance()->getPos()._y);
    Motor_Intaker1.resetPosition();
    moveIntaker(100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) >= 130);
    moveIntaker(0);
    Chassis::getInstance()->chassisBrake(brakeType::coast);
    timerForwardWithHeading(-40, 250, 0);

    // 后退一步防止蹭�?
    // quickMoveToWithHeading(44.6245, 35.74217, 265.185, 100);

    quickMoveToWithoutStop(108, 20, 290, 100);

    // // 前进到导入台，清空
    // setFlyWheelSpeed(340);
    // moveToWithHeading(154 - 5, 10, 262, 100);
    // timerMove(Vector(-50, 0), 300);
    // // Position::getInstance()->setGlobalPosition(Position::getInstance()->getPos()._x, 0);
    // this_thread::sleep_for(100);
    // aimAt(283, 20, -1);
    // moveIntaker(100);

    // shoot(3);
    // for(int i = 1; i <= 9; ++i){
    //     this_thread::sleep_for(300);
    //     shoot();
    // }
    // moveIntaker(0);


    // 吃三斜
    moveIntaker(100);
    quickMoveToWithHeading(176, 121, 41, 100);
    timerForwardWithHeading(90, 2000, 0);
    // quickMoveToWithHeading(176 + 80, 121 + 80, 41, 100);
    // moveToWithHeading(237.324 + 20, 177.755 + 20, 45, 100);

    setFlyWheelSpeed(335);
    moveToWithHeading(250.346, 140 + 15, 0, 100);
    aimAt(283, 20, -1);

    shoot(3);


    // quickMoveToWithHeading(315 - 15, 154.0409, 11.467, 100);
    // setFlyWheelSpeed(335);
    // quickMoveToWithHeading(315, 94.0409 + 10, 11.467, 100);
    // Piston_Angler.set(true);
    // setAimingStatus(true);
    // this_thread::sleep_for(700);
    // shoot(3);
    // setAimingStatus(false);
    // Piston_Angler.set(false);
    // quickMoveToWithHeading(315 - 15, 154.0409, 11.467, 100);


    // 吃三�?
    moveIntaker(100);
    // quickMoveToWithHeading(188.381, 154.0409, 100, 100);
    quickMoveToWithHeading(222, 120, 135, 100);
    quickMoveToWithHeading(273 + 10, 120, 135, 100);

    // setFlyWheelSpeed(325);
    // moveToWithHeading(298.406, 138.736, 10.2211, 100);
    // setAimingStatus(true, 8);
    // this_thread::sleep_for(700);
    // shoot(3);
    // setAimingStatus(false);
    quickMoveToWithHeading(315, 154.0409, 11.467, 100);
    setFlyWheelSpeed(350);
    moveToWithHeading(315, 94.0409 + 20, 11.467, 100);
    Piston_Angler.set(true);
    aimAt(283, 20, -1);

    shoot(3);

    Piston_Angler.set(false);
    quickMoveToWithHeading(315 - 15, 154.0409, 11.467, 100);
    quickMoveToWithHeading(298.406 - 15, 160.736, 10.2211, 100);


    // 吃三�?1
    quickMoveToWithHeading(189.461 + 10, 137.619 - 10, 240.076, 100);
    moveIntaker(100);
    quickMoveToWithHeading(146, 95, 226, 100);
    timerForwardWithHeading(50, 1500, 0);

    setFlyWheelSpeed(350);
    moveToWithHeading(222.48 - 2, 11.1852, 264.925, 100);
    Piston_Angler.set(true);
    aimAt(283, 20, -1);

    shoot(3);

    Piston_Angler.set(false);


    // 吃三�?2
    quickMoveToWithHeading(88, 59, 310, 100);
    moveIntaker(100);
    timerForwardWithHeading(30, 1500, 0);

    setFlyWheelSpeed(350);
    moveToWithHeading(222.48 - 2, 11.1852, 264.925, 100);
    Piston_Angler.set(true);
    aimAt(283, 20, -1);

    shoot(3);

    Piston_Angler.set(false);

    quickMoveToWithHeading(30, 45, 270, 100);
    moveToWithHeading(30, 30, 225, 100);
    // Piston_Deployer.set(true);

    this_thread::sleep_for(2000);*/

    cout << "ending autonomous" << endl;
    // ChassisControl.interrupt();
    while(true){
        // cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
        this_thread::sleep_for(100);
    }
}
#endif

// void autonomous(){
//     cout << "enter auto functions" << endl;
//     // Inertial.setHeading(0, rotationUnits::deg);
//     Position *p = Position::getInstance();
//     Chassis *c = Chassis::getInstance();
//     setFlyWheelSpeed(300);
//     moveIntaker(100);
//     this_thread::sleep_for(1000);

//     setFlyWheelSpeed(400);
//     moveToWithHeading(60.1522, 23.597, 312.231, 80);
//     intakeDisc();
//     aimAt(20, 300);
//     setAimingStatus(true);
//     // turnTo()
//     shoot(3);
//     setAimingStatus(false);
//     this_thread::sleep_for(300);

//     setFlyWheelSpeed(380);
//     // moveToWithHeading(87.9921, 49.9452, 320.766, 80);
//     intake3Discs(89.692, 52.7819, 313.485);
//     timerForward(-30, 500);
//     aimAt(20, 300);
//     shoot(3);
//     this_thread::sleep_for(300);

//     setFlyWheelSpeed(380);
//     // moveToWithHeading(113.623, 50.6454, 48.1995, 80);
//     intake3Discs(106.437, 54.3264, 53.6581);
//     aimAt(50, 310);
//     shoot(3);
//     this_thread::sleep_for(300);

//     moveToWithHeading(30, 30, 0, 100);

//     setFlyWheelSpeed(0);
//     moveIntaker(0);

//     cout << "ending autonomous" << endl;
//     // ChassisControl.interrupt();
//     while(true){
//         cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;
//         this_thread::sleep_for(100);
//     }
// }

#ifdef COMPETITION_LEFT

void autonomous(){

    cout << "enter auto functions" << endl;
    // Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();
    setFlyWheelSpeed(410);
    aimPreciselyAt(45,300, -2);
    this_thread::sleep_for(1000);
    shoot(2);
    //发射两个预装disc
    
    setFlyWheelSpeed(387);
    intake3DiscsWithPiston(95,57,315);
    moveIntaker(0);
    // timerForwardWithHeading(-50,300, 0);
    moveToWithHeading(92, 64, 315, 60);
    aimPreciselyAt(45, 300, -8);
    shoot(3);
    //吃中线上三个盘子并发�?

    setFlyWheelSpeed(390);
    intake3DiscsWithoutPiston(103, 40, 45);
    this_thread::sleep_for(700);
    aimPreciselyAt(45, 300, -1);
    shoot(3);
    //吃右侧三个盘子并发射

    setFlyWheelSpeed(380);
    moveIntaker(100);
    moveToWithHeading(124, 79, 315, 80);
    timerForwardWithHeading(28, 350, 0);
    this_thread::sleep_for(300);
    timerForwardWithHeading(-50, 500, 0);

    moveToWithHeading(151, 110, 315, 80);
    timerForwardWithHeading(28, 300, 0);
    this_thread::sleep_for(300);
    timerForwardWithHeading(-50, 500, 0);

    moveToWithHeading(178, 123, 45, 80);
    this_thread::sleep_for(300);
    aimPreciselyAt(45, 300, -2);
    moveIntaker(0);
    shoot(3);
    
    //吃中线两个盘子和右侧一组盘子中最靠左盘子，并发射

    setFlyWheelSpeed(380);
    moveIntaker(100);
    moveToWithHeading(193,108,135,80);
    moveToWithHeading(193,30,135,60);
    this_thread::sleep_for(300);
    moveToWithHeading(180,108,135,80);
    moveIntaker(0);
    aimPreciselyAt(45, 300, -3);
    shoot(3, 50);
    //吃barrier左侧三个盘子，并发射
    
    moveToWithHeading(75,13,180,80);
    timerForwardWithHeading(28, 300, 0);
    moveIntaker(30);
    this_thread::sleep_for(400);
    moveIntaker(0);
    this_thread::sleep_for(300);
    timerForwardWithHeading(-35, 300, 0);


    setFlyWheelSpeed(0);

}

#endif

#ifdef COMPETITION_RIGHT
void autonomous(){
    cout << "enter auto functions" << endl;
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();

    /*--------------------------------------*/
    /*--------------------------------------*/

    // /*------------------------------------------------------------------------------------------*/
    // 抢三盘

    setFlyWheelSpeed(435);
    moveIntaker(70);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(52+2+2, 71+2-2, 125, 100);
    // timerForwardWithHeading(50, 100, 0);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(1000 + 200);

    aimAt(283, 28, 4-0.6+0.8);
    
    //  shoot(3);
    #ifdef SHoot
    shoot(3,550); 
    #endif

    /*------------------------------------------------------------------------------------------*/

    quickTurnTo(98,80);

    //吃中线
    setFlyWheelSpeed(423);
    moveToWithHeading(61.4+25+30-4+3, 76.7+25+30+4+3, 98, 60);
    
    quickTurnTo(348-20,100);
    
    //吃近中线一个
    timerForwardWithHeading(50, 400, 0);
    this_thread::sleep_for(800);

    aimAt(283, 28, +2.5);

    #ifdef SHoot
    shoot(3,500);
    #endif

    /*------------------------------------------------------------------------------------------*/

    //三竖
    setFlyWheelSpeed(424);
    moveIntakerWithRPM(400);
    quickMoveToWithHeading(102.011 + 10, 200.235 - 15-1, 322.051, 100);
    quickMoveToWithHeading(102.011 - 60, 200.235 - 10-1, 322.051, 60);

    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        fourBarFlag = false;
    }
    #endif  

    quickMoveToWithHeading(54+75-40 , 79+55+20 , 325, 80); 
    this_thread::sleep_for(500);   

    aimAt(283, 28, -1 + 3);

    #ifdef SHoot
    shoot(3,800);
    #endif

    /*------------------------------------------------------------------------------------------*/

    // //吃中线
    setFlyWheelSpeed(440);

    quickMoveToWithHeading(54+12-12-3, 79+65-12+12-3, 45-180, 80); 
    this_thread::sleep_for(800);

    Piston_IntakerLifter.set(true);
    //吃两个预装
    quickMoveToWithHeading(56-18, 141+8+5 , -90 , 80);
    timerForwardWithHeading(60, 160 ,0);
    this_thread::sleep_for(200);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(700);


    #ifdef FourBarProcess
    if (fourBarFlag)
    {
        shoot();
        fourBarFlag = false;
    }
    #endif
    quickMoveToWithHeading(56+12+10, 141+8+5+4 , -90 , 80);
    aimAt(283, 28, +5);

    #ifdef SHoot
    shoot(3,850);
    #endif

    // /*------------------------------------------------------------------------------------------*/

    moveIntaker(0);
    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
    quickMoveToWithHeading(16, 42+18, 280, 100);
    timerForwardWithHeading(50, 450, 0);
    Chassis::getInstance()->chassisBrake(brakeType::hold);
    Chassis::getInstance()->autoSetRobotVel(Vector(0, 20), 20);
    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -50);  // 转动Roller
    moveIntaker(0);
    timerForwardWithHeading(-50, 200, 0);

    // /*------------------------------------------------------------------------------------------*/
    moveIntakerWithRPM(400);
    // setFlyWheelSpeed(440);
    moveToWithHeading(61.4-30-4+3, 76.7-30+4+3, 126, 60);
    // moveToWithHeading(61.4-30-4+3-30, 76.7-30+4+3-30, 126, 60);

    // aimAt(283, 28, +5); 

    // #ifdef SHoot
    // shoot(2,800);
    // #endif

    // moveToWithHeading(25.707, 93.893, 90, 100);     


    setFlyWheelSpeed(0);
    cout << "ending autonomous" << endl;



    while(true){
        this_thread::sleep_for(100);
    }
}
#endif

#ifdef COMPETITION_RIGHT_Oliver
void autonomous(){
    cout << "enter auto functions" << endl;
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();

    //抢三盘    
    setFlyWheelSpeed(432);
    moveIntaker(90);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(52+2+2, 71+2-2, 125, 100);
    Piston_IntakerLifter.set(false);
    this_thread::sleep_for(1200);

    timerForwardWithHeading(-80, 150, 0);
    this_thread::sleep_for(300);

    aimAt(283, 28, +2);
    this_thread::sleep_for(500);

     // shoot(3);
    #ifdef SHoot
    shoot(3,500);
    #endif

    /*------------------------------------------------------------------------------------------*/
    
    //86.059,100.946,98.32
    //118.3,134.6,97.7
    //103.38,147.64,346.7
    
    quickTurnTo(108,100);

    //吃中线
    setFlyWheelSpeed(422);
    quickMoveToWithHeading(89 , 105 , 113, 80);
    this_thread::sleep_for(800);  
    quickMoveToWithHeading(119 , 135+2 , 113, 80); 
    this_thread::sleep_for(800);
    
    quickTurnTo(348-15,100);
    
    //吃近中线一个
    timerForwardWithHeading(50, 400, 0);
    this_thread::sleep_for(800);

    aimAt(283, 28, +2.5);

    #ifdef SHoot
    shoot(3,500);
    #endif

    /*------------------------------------------------------------------------------------------*/
    
    //93.31,201.95,300.19
    //55.62,199.64,295
    //96.30,136.72,301.29
    
    //三竖
    setFlyWheelSpeed(421);
    moveIntaker(90);
    quickMoveToWithHeading(97.45, 201.64 -2, 311.61, 100);
    quickMoveToWithHeading(48.43 -4 , 201.64 -2, 311.61, 100);

    quickMoveToWithHeading(97.94 , 155.01 , 297.96, 100); 
    this_thread::sleep_for(500);   

    aimAt(283, 28, +2.5);

    #ifdef SHoot
    shoot(3,500);
    #endif

    /*------------------------------------------------------------------------------------------*/
    
    //76.965,120.24,299.76
    //41.73,151.37,289.44
    //61.99,108.62,287.95
    
    setFlyWheelSpeed(425);
    quickTurnTo(250,100);
    
    //吃中线下一个
    moveIntaker(90);
    timerForwardWithHeading(60,400,0);
    this_thread::sleep_for(700);
    
    //吃两个预装
    quickMoveToWithHeading(45.9 , 139.63+10 , 264.2, 100);
    timerForwardWithHeading(80, 350, 0);

    this_thread::sleep_for(700);
    
    quickMoveToWithHeading(97.94 -20 , 155.01 -20 , 297.96 + 10, 100); 
    this_thread::sleep_for(500);   

    aimAt(283, 28, +2.5);

    #ifdef SHoot
    shoot(3,500);
    #endif
    /*------------------------------------------------------------------------------------------*/

    moveIntaker(0);
    quickMoveToWithHeading(16, 65, 270, 100);

    this_thread::sleep_for(300); 

    timerForwardWithHeading(50, 450, 0);    // 前移靠近Roller
    Chassis::getInstance()->chassisBrake(brakeType::hold);

    Motor_Intaker1.resetPosition();
    moveIntaker(-100);
    waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -50 + 5);  // 转动Roller
    moveIntaker(0);

    timerForwardWithHeading(-50, 200, 0);
    
    /*------------------------------------------------------------------------------------------*/
    
    //40.54,47.38,154.56
    //16.92,16.37,153.33
    //43.32,76.11,283.27    
    
    setFlyWheelSpeed(425);
    moveIntaker(90);
    quickTurnTo(143.62,100);
    
    quickMoveToWithHeading(36 -5, 44 -2, 143.62 , 60);
    this_thread::sleep_for(300);
    
    quickMoveToWithHeading(7.04 , 21.38 -2 , 143.62 , 60);
    this_thread::sleep_for(300);
    
    quickTurnTo(90,100);
    
    quickMoveToWithHeading(110 - 20, 125 - 12, 90, 100); 
    aimAt(283, 28, +2.5);
    
    #ifdef SHoot
    shoot(3,500);
    #endif
    
    quickTurnTo(90,100);

    setFlyWheelSpeed(0);
    cout << "ending autonomous" << endl;

    while(true){
        this_thread::sleep_for(100);
    }
}
#endif

#ifdef FIFTEEN
void autonomous(){
    // double newhead = IMUHeading() + 180;
    // while (newhead > 360) newhead -= 360;
    // while (newhead < 0) newhead += 360;
    // Inertial.setHeading(newhead, rotationUnits::deg);
    cout << "enter auto functions" << endl;
    // Inertial.setHeading(0, rotationUnits::deg);
    Position *p = Position::getInstance();
    Chassis *c = Chassis::getInstance();

    setFlyWheelSpeed(405);
    // this_thread::sleep_for(600);
    // moveToWithHeading(45.0011 - 8, 93.8312 + 8, 292, 80);
    // // aimAt(283, 20);
    // shoot(2, 100);

    // timerForwardWithHeading(100,100,0);
    // turnTo(125, 100);
    moveIntakerWithRPM(350);
    Piston_IntakerLifter.set(true);
    quickMoveToWithHeading(57, 76, 125, 100);
    // timerForwardWithHeading(50, 100, 0);
    Piston_IntakerLifter.set(false);
    // intake3DiscsWithPiston(54.8055 - 5, 86.5308 + 5, 133.152);
    this_thread::sleep_for(500);


    // quickMoveToWithHeading(45.0011 - 8, 93.8312 + 8, 294.321, 80);
    // this_thread::sleep_for(200);
    
    setFlyWheelSpeed(405);
    timerForwardWithHeading(-40, 200, 0);
    // aimPreciselyAt(283, 20, -2);
    aimAt(283, 20, 1);
    // this_thread::sleep_for(100);
    shoot(3, 100);


    // Move Roller
    moveIntaker(0);
    quickMoveToWithHeading(13, 75, -90, 80);
    timerForwardWithHeading(50, 300, 0);
    moveIntaker(-30);
    this_thread::sleep_for(450);
    moveIntaker(0);
    timerForwardWithHeading(-50, 150, 0);





    // timerForwardWithHeading(100, 100, 0);
    // quickTurnTo(160);
    // moveIntaker(100);
    // quickMoveToWithHeading(27, 49, 133, 100);
    // timerForwardWithHeading(60, 200, 0);
    // // this_thread::sleep_for(300);
    // timerForwardWithHeading(-100, 100, 0);
    moveIntaker(100);
    quickMoveToWithHeading(61.398, 111.639, 41.840, 80);
    quickMoveToWithHeading(100.334, 144.635, 47.111, 100);
    // quickMoveToWithHeading(127.938, 128.438, 132.648, 100);
    // timerForwardWithHeading(100, 100, 0);
    // quickMoveToWithHeading(59.2527 - 2 + 10, 123.568 + 8, 64.9549, 100);
    // this_thread::sleep_for(300);
    // quickMoveToWithHeading(87.2679 + 1 + 10 - 4, 112.848 + 8 - 4, 124.576, 100);
    // this_thread::sleep_for(300);
    
    setFlyWheelSpeed(395);
    // timerForwardWithHeading(-100, 150, 0);
    // aimPreciselyAt(283, 20, -3);
    aimAt(283, 20, 1);
    // this_thread::sleep_for(200);
    shoot(3, 100);


    setFlyWheelSpeed(450);
    moveIntaker(80);
    quickMoveToWithHeading(102.011 + 10, 200.235 - 15, 322.051, 100);
    // this_thread::sleep_for(200);
    quickMoveToWithHeading(102.011 - 70, 200.235 - 10, 322.051, 100);

    // setFlyWheelSpeed(420);
    //slow
    // quickMoveToWithHeading(45.0011 - 8, 93.8312 + 8, 294.321, 80);
    // aimPreciselyAt(283, 20, -3);
    aimAt(283, 20, 1);
    shoot(3, 150);


    //
    
    setFlyWheelSpeed(0);
    cout << "ending autonomous" << endl;



    while(true){
        this_thread::sleep_for(100);
    }
}
#endif

// /*debug*/
    // testFlyWheelFlag = true;

    // setFlyWheelSpeed(326);//360

    // this_thread::sleep_for(5000);

    // testFlyWheelFlag = false;

    // /*------------------------------------------------------------------------------------------*/
    // //抢三盘
    // // testFlyWheelFlag = true;

    // setFlyWheelSpeed(390);
    // moveIntaker(70);
    // Piston_IntakerLifter.set(true);
    // quickMoveToWithHeading(52+2, 71+2, 125, 100);
    // // timerForwardWithHeading(50, 100, 0);
    // Piston_IntakerLifter.set(false);
    // this_thread::sleep_for(1000 + 200);

    // aimAt(283, 28, 4-1.1);
    
    // //  shoot(3);
    // #ifdef SHoot
    // shoot(1,400); 
    // setFlyWheelSpeed(400);
    // this_thread::sleep_for(100);
    // shoot(1,400); 
    // setFlyWheelSpeed(390);
    // this_thread::sleep_for(100);
    // shoot(1,400);
    // #endif

    // // testFlyWheelFlag = false;

    // /*------------------------------------------------------------------------------------------*/
    
    // setFlyWheelSpeed(385);
    // moveIntaker(70);
    // //吃内线一个盘
    // quickMoveToWithHeading(56, 79+40, 45, 80);
    // this_thread::sleep_for(700);
    // quickMoveToWithHeading(54+38+2-3, 79+18+2+3, 125, 80);
    // this_thread::sleep_for(800);

    // #ifdef FourBarProcess
    // if (fourBarFlag)
    // {
    //     shoot();
    //     fourBarFlag = false;
    // }
    // #endif   

    // quickMoveToWithHeading(56+15, 79+40-15, -45, 80);
    // aimAt(283, 28, 2.25-0.3);

    // #ifdef SHoot
    // shoot(1,700);
    // setFlyWheelSpeed(385);
    // shoot(1,700);
    // #endif

    // /*------------------------------------------------------------------------------------------*/

    // //吃中线
    // setFlyWheelSpeed(395-10);

    // Piston_IntakerLifter.set(true);
    // //吃两个预装
    // quickMoveToWithHeading(56-8, 141+8-1 , -90 , 80);
    // timerForwardWithHeading(60, 220 ,0);
    // this_thread::sleep_for(200);
    // Piston_IntakerLifter.set(false);
    // this_thread::sleep_for(700);

    // quickMoveToWithHeading(54+40, 79+65, 45, 60); 
    // this_thread::sleep_for(800);

    // #ifdef FourBarProcess
    // if (fourBarFlag)
    // {
    //     shoot();
    //     fourBarFlag = false;
    // }
    // #endif

    // aimAt(283, 28, -1 + 3 - 1.5-0.5);

    // #ifdef SHoot
    // shoot(1,800);
    // setFlyWheelSpeed(380);
    // shoot(1,800);
    // setFlyWheelSpeed(380);
    // shoot(1,800);
    // #endif

    // /*------------------------------------------------------------------------------------------*/
    // //三竖
    // setFlyWheelSpeed(395 - 20 + 10 - 2.5 - 2.5 - 2.5);
    // moveIntaker(70);
    // quickMoveToWithHeading(102.011 + 10, 200.235 - 15-2, 322.051, 100);
    // quickMoveToWithHeading(102.011 - 70, 200.235 - 10-2, 322.051, 100);

    // #ifdef FourBarProcess
    // if (fourBarFlag)
    // {
    //     shoot();
    //     fourBarFlag = false;
    // }
    // #endif

    // quickMoveToWithHeading(54+75-20 , 79+55+20 , 325, 60); 
    // this_thread::sleep_for(500);   

    // aimAt(283, 28, -1 + 3 - 1);

    // #ifdef SHoot
    // shoot(3,800);
    // #endif
    // // shoot(1); //
    // // setFlyWheelSpeed(395 - 20 - 20 + 20);
    // // this_thread::sleep_for(100);
    // // shoot(1); //
    // // setFlyWheelSpeed(395 - 20 + 20 + 20);
    // // this_thread::sleep_for(100);
    // // shoot(1); //

    // /*------------------------------------------------------------------------------------------*/
    // // moveIntaker(80);
    // // quickMoveToWithHeading(54-32, 79-35, 125, 80);
    // // timerForwardWithHeading(60, 150, 0);
    // // this_thread::sleep_for(1000);
    // // moveIntaker(0);

    // /*------------------------------------------------------------------------------------------*/

    // moveIntaker(0);
    // quickMoveToWithHeading(16, 42+23, 270, 100);

    // this_thread::sleep_for(1000); 

    // timerForwardWithHeading(50, 450, 0);    // 前移靠近Roller
    // Chassis::getInstance()->chassisBrake(brakeType::hold);
    // Motor_Intaker1.resetPosition();
    // moveIntaker(-100);
    // waitUntil(Motor_Intaker1.position(rotationUnits::deg) <= -50);  // 转动Roller
    // moveIntaker(0);
    // timerForwardWithHeading(-50, 200, 0);

    // moveToWithHeading(25.707, 93.893, 90, 100); 