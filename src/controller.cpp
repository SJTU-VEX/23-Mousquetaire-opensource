#ifndef CONTROLLER_H_
#define CONTROLLER_H_
#include "controller.h"
#include "robot-config.h"

#include <stdlib.h>
#include <iostream>

// Define the buttons
int t, A1, A2, A3, A4, L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN,
            last_L1, last_L2, last_R1, last_R2, 
            last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;

bool press_X = false, press_Y = false, press_A = false, press_B = false, press_UP = false, press_DOWN = false, press_LEFT = false, press_RIGHT = false;
bool release_L2 = false, press_L2 = false;

/**
 * @brief 更新手柄按键和摇杆的输入
 * 
 */
void defineController(){
    while (true)
    {
        last_L1 = L1;
        last_L2 = L2;
        last_R1 = R1;
        last_R2 = R2;
        last_X = X;
        last_Y = Y;
        last_A = A;
        last_B = B;
        last_LEFT = LEFT;
        last_RIGHT = RIGHT;
        last_UP = UP;
        last_DOWN = DOWN;
        t = Brain.timer(vex::timeUnits::msec);
        A1 = Controller.Axis1.position(vex::percentUnits::pct);
        A2 = Controller.Axis2.position(vex::percentUnits::pct);
        A3 = Controller.Axis3.position(vex::percentUnits::pct);
        A4 = Controller.Axis4.position(vex::percentUnits::pct);
        L1 = Controller.ButtonL1.pressing();
        L2 = Controller.ButtonL2.pressing();
        R1 = Controller.ButtonR1.pressing();
        R2 = Controller.ButtonR2.pressing();
        X = Controller.ButtonX.pressing();
        Y = Controller.ButtonY.pressing();
        A = Controller.ButtonA.pressing();
        B = Controller.ButtonB.pressing();
        LEFT = Controller.ButtonLeft.pressing();
        RIGHT = Controller.ButtonRight.pressing();
        UP = Controller.ButtonUp.pressing();
        DOWN = Controller.ButtonDown.pressing();

        // pressFlag
        if (X && !last_X) press_X = true;
        if (A && !last_A) press_A = true;
        if (B && !last_B) press_B = true;
        if (Y && !last_Y) press_Y = true;
        if (UP && !last_UP) press_UP = true;
        if (DOWN && !last_DOWN) press_DOWN = true;
        if (RIGHT && !last_RIGHT) press_RIGHT = true;
        if (LEFT && !last_LEFT) press_LEFT = true;
        if (L2 && !last_L2) press_L2 = true;
        if (last_L2 && !L2) release_L2 = true;
        // std::cout<<DOWN<<' '<<last_DOWN<<std::endl;

        this_thread::sleep_for(10);
    }
}

#endif