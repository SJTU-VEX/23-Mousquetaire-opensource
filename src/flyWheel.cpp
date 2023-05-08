#include "vex.h"
#include "robot-config.h"
#include "flyWheel.h"
#include "parameters.h"
#include <iostream>
using namespace std;
double FlyWheel_Error, Last_FlyWheel_Error;
double current_Speed;
float FLYWHEEL_SPEED = 0;
float LAST_FLYWHEEL_SPEED = 0;

/**
 * voltage is from +100 to -100, and velocity is from 0 to 600.
*/
double FlywheelVoltagetoVelocity(double percent)
{
    return (11.711 * percent + 26.004) / 2;
}

/**
 * voltage is from +100 to -100, and velocity is from 0 to 600.
*/
double FlywheelVelocitytoVoltage(double speed)
{
    return (speed - 26.004 / 2) / (11.711 / 2);
}

/**
 * @param percent is from +100 to -100.
*/
void turnFlyWheelVoltage(float percent)
{
    Motor_FlyWheel2.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
    Motor_FlyWheel1.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
    return;
}

/**
 * @return flywheel speed from 0 to 600.
*/
void FlyWheelSpeed()
{
    current_Speed = (Motor_FlyWheel1.velocity(rpm) + Motor_FlyWheel2.velocity(rpm)) / 2;
}

void setFlyWheelSpeed(double Target_Speed){
    FLYWHEEL_SPEED = Target_Speed;
}

void updateFlyWheelSpeed(double Target_Speed)
{
    // cout<<Target_Speed<<endl;
    FlyWheel_Error = Target_Speed - current_Speed;
    if (Target_Speed == 0)
    {
        turnFlyWheelVoltage(0);
        return;
    }
    else
    {
        if (FlyWheel_Error >= Flywheel_P_Control_Domain)
        {
            turnFlyWheelVoltage(100);
            // cout<<"1"<<endl;
        }

        if (FlyWheel_Error < Flywheel_P_Control_Domain && FlyWheel_Error >= 0)
        {   
            turnFlyWheelVoltage(Target_Speed + kp * FlyWheel_Error);
            // cout<<"2"<<endl;
        }

        if (FlyWheel_Error < -Flywheel_P_Control_Domain)
        {
            turnFlyWheelVoltage(FlywheelVelocitytoVoltage(Target_Speed));
            // cout<<"3"<<endl;
        }

        // cout<<FlyWheel_FlyWheel_Error<<endl;
        return;
    }
}

void OpenLoopSetFlyWheelSpeed(double TargetPercent){
    turnFlyWheelVoltage(TargetPercent);
}

static bool is_first_time = true;
static float tbh = 0, output = 0, Kp = 0.1;

void updateTBHcontroller() {
    FlyWheel_Error = FLYWHEEL_SPEED - current_Speed;

    if(FLYWHEEL_SPEED != LAST_FLYWHEEL_SPEED) {
        is_first_time = 1;
    }
    output += Kp * FlyWheel_Error;
    if(output > 100) {
        output = 100;
    }
    if(output < 0) {
        output = 0;
    }
    if(sign(FlyWheel_Error) != sign(Last_FlyWheel_Error)) {
        if(is_first_time) {
            output = FlywheelVelocitytoVoltage(FLYWHEEL_SPEED);
            is_first_time = 0;
        }
        else {
            output = (output + tbh) / 2;
        }
        tbh = output;
    }
    Last_FlyWheel_Error = FlyWheel_Error;
    LAST_FLYWHEEL_SPEED = FLYWHEEL_SPEED;
    // cout << tbh << ", " << output << ", " << current_Speed << endl;
    turnFlyWheelVoltage(output);
}

void updateFlyWheel(){
    while(true){
        FlyWheelSpeed();
        // updateFlyWheelSpeed(FLYWHEEL_SPEED);
        updateTBHcontroller();
        if (testFlyWheelFlag)
        {
            cout<<current_Speed<<endl;
        }
        this_thread::sleep_for(10);
    }
}