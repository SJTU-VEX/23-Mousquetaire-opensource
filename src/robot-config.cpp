#include "vex.h"
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
inertial Inertial = inertial(PORT10);

vex::distance Distance = vex::distance(PORT5);

motor Motor_BaseLFU = motor(PORT9, ratio18_1, true);
motor Motor_BaseLFD = motor(PORT8, ratio18_1, false);
motor Motor_BaseLBU = motor(PORT7, ratio18_1, true);
motor Motor_BaseLBD = motor(PORT6, ratio18_1, false);

motor Motor_BaseRBU = motor(PORT2, ratio18_1, true);
motor Motor_BaseRBD = motor(PORT1, ratio18_1, false);
motor Motor_BaseRFU = motor(PORT4, ratio18_1, true);
motor Motor_BaseRFD = motor(PORT3, ratio18_1, false);

controller Controller = controller(primary);

// triport Expander = triport(PORT8);

// encoder EncoderL = encoder(Brain.ThreeWirePort.A);
// encoder EncoderR = encoder(Brain.ThreeWirePort.E);
encoder EncoderL = encoder(Brain.ThreeWirePort.G);
encoder EncoderR = encoder(Brain.ThreeWirePort.A);

motor Motor_Intaker1 = motor(PORT13, ratio18_1, false);
motor Motor_Intaker2 = motor(PORT20, ratio18_1, true);
motor Motor_FlyWheel1 = motor(PORT12, ratio6_1, true);
motor Motor_FlyWheel2 = motor(PORT11, ratio6_1, false);

digital_out Piston_Trigger = digital_out(Brain.ThreeWirePort.E);
digital_out Piston_Angler = digital_out(Brain.ThreeWirePort.D);
digital_out Piston_Deployer = digital_out(Brain.ThreeWirePort.C);
digital_out Piston_IntakerLifter = digital_out(Brain.ThreeWirePort.F);

vision VisionSensor = vision (PORT15, 105, VisionSensor__SIG_BLUE, VisionSensor__SIG_RED);
//red - 61 blue - 105
signature VisionSensor__SIG_BLUE = signature (1, -2919, -1657, -2288, 7423, 10879, 9150, 3.2, 0);
// signature VisionSensor__SIG_RED = signature(2, 9601, 10385, 9993, -3121, 815, -1153, 3.7, 0);
signature VisionSensor__SIG_RED = signature (2, 7493, 8467, 7980, -2091, -1225, -1658, 5.6, 0);
// digital_out Piston_Deployer1 = digital_out(Brain.ThreeWirePort.A);
// digital_out Piston_Deployer2 = digital_out(Brain.ThreeWirePort.A);
// digital_out Piston_Deployer3 = digital_out(Brain.ThreeWirePort.A);
// digital_out Piston_Deployer4 = digital_out(Brain.ThreeWirePort.A);
// digital_out Piston_Deployer5 = digital_out(Brain.ThreeWirePort.A);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void){
    // Inertial.calibrate();
    // std::cout << "calibrated!" << std::endl;
    // while (Inertial.isCalibrating());
    // std::cout << "calibrated2!" << std::endl;
    // this_thread::sleep_for(3000);
}
