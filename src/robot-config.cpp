#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

controller controller1 = controller(primary);
motor fl = motor(PORT6, ratio18_1, true);
motor ml = motor(PORT5, ratio18_1, true);
motor bl = motor(PORT4, ratio18_1, true);
motor fr = motor(PORT1, ratio18_1, false);
motor mr = motor(PORT2, ratio18_1, false);
motor br = motor (PORT3, ratio18_1, false);
motor intake = motor(PORT9, ratio18_1, false);
inertial inertialSensor = inertial(PORT11);
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
  //TurnGyroSmart.startCalibration(1);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
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