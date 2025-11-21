#include "vex.h"
#include "robot-config.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

//drivetrain motors
controller controller1 = controller(primary);
motor fl = motor(PORT9, ratio6_1, true);  
motor ml = motor(PORT8, ratio6_1, true); //backwards
motor bl = motor(PORT7, ratio6_1, true); 
motor fr = motor(PORT3, ratio6_1, false); // working
motor mr = motor(PORT2, ratio6_1, false); // working
motor br = motor(PORT1, ratio6_1, false); // working
// intake motors
motor intake = motor(PORT4, ratio6_1, false); // BOTTOM INTAKE
motor intake2 = motor(PORT10, ratio6_1, true); // TOP INTAKE
//motor intake3 = motor(PORT20, ratio18_1, true); // INSIDE INTAKE
pneumatics descore = pneumatics(Brain.ThreeWirePort.C);
pneumatics matchloader = pneumatics(Brain.ThreeWirePort.A);
inertial inertialSensor = inertial(PORT5); 

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