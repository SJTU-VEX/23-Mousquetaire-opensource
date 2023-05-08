#include "distance.h"
#include "vex.h"
#include "robot-config.h"
#include "my-timer.h"
#include "parameters.h"
#include <iostream>
using namespace vex;

void updateDistance(){
    MyTimer mytimer;
    // bool flag = false;
    while(true){
        if (!fourBarFlag){
            if (Distance.isObjectDetected() && Distance.objectDistance(distanceUnits::cm) <= 8 && mytimer.getTime() > 1100){
                // flag = true;
                fourBarFlag = true;
                // std::cout << "object detected!" << std::endl;
            }
        }
        else{
            if (!Distance.isObjectDetected() || Distance.objectDistance(distanceUnits::cm) >= 15){
                fourBarFlag = false;
                // double sz = 14;
                // double v = sz / mytimer.getTime() * 1000;
                mytimer.reset();
                // std::cout << v << std::endl;
            }
        }
        this_thread::sleep_for(5);
    }
}