#include "vex.h"
#include "robot-config.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

controller controller1 = controller(primary);
motor fl = motor(PORT20, ratio6_1, true); 
motor ml = motor(PORT19, ratio6_1, true); 
motor bl = motor(PORT18, ratio6_1, true); 
motor fr = motor(PORT12, ratio6_1, false); 
motor mr = motor(PORT13, ratio6_1, false); 
motor br = motor(PORT14, ratio6_1, false); 
// motor fl(PORT20, ratio6_1, true);
// motor ml(PORT19, ratio6_1, true);
// motor bl(PORT18, ratio6_1, true);
// motor fr(PORT12, ratio6_1, false);
// motor mr(PORT13, ratio6_1, false);
// motor br(PORT14, ratio6_1, false);
motor intake = motor(PORT5, ratio6_1, true); // BOTTOM AND MIDDLE INTAKE
motor intake2 = motor(PORT2, ratio18_1, true); // TOP INTAKE
motor intake3 = motor(PORT3, ratio18_1, false); // INSIDE INTAKE
inertial inertialSensor = inertial(PORT1); 

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  // TurnGyroSmart.startCalibration(1);
  // Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  // while (TurnGyroSmart.isCalibrating()) {
  //   wait(25, msec);
  // }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}

// motor* driveMotors[] = {
//   &fl, &ml, &bl,
//   &fr, &mr, &br

// };