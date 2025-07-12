#include "vex.h"
#include "robot-config.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

controller controller1 = controller(primary);
motor fl = motor(PORT19, ratio6_1, false); 
motor ml = motor(PORT20, ratio6_1, false); 
motor bl = motor(PORT10, ratio6_1, false); 
motor fr = motor(PORT12, ratio6_1, true); 
motor mr = motor(PORT11, ratio6_1, true); 
motor br = motor (PORT2, ratio6_1, true); 
motor intake = motor(PORT6, ratio6_1, true); // bottom and middle intake
motor intake2 = motor(PORT9, ratio18_1, true); // top intake
motor intake3 = motor(PORT7, ratio18_1, true); // inside intake
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