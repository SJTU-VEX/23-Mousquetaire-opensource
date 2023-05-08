#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_
#include "vex.h"

using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial Inertial;
// Base motors
// Left/Right Front/Back Up/Down
extern motor Motor_BaseLFU;
extern motor Motor_BaseLFD;
extern motor Motor_BaseLBU;
extern motor Motor_BaseLBD;
extern motor Motor_BaseRBU;
extern motor Motor_BaseRBD;
extern motor Motor_BaseRFU;
extern motor Motor_BaseRFD;

extern motor Motor_Intaker1;
extern motor Motor_Intaker2;
extern motor Motor_FlyWheel1;
extern motor Motor_FlyWheel2;


extern controller Controller;

extern digital_out Piston_Trigger;
extern digital_out Piston_Angler;
extern digital_out Piston_Deployer;
extern digital_out Piston_IntakerLifter;

extern vex::distance Distance;

extern triport Expander;
extern encoder EncoderL;
extern encoder EncoderR;

extern vision VisionSensor;
extern vision::signature VisionSensor__SIG_BLUE;
extern vision::signature VisionSensor__SIG_RED;
extern vision::signature VisionSensor__SIG_GREEN;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );

#endif