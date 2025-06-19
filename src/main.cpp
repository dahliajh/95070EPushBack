/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
void moveAllWheels(int SpeedLeft, int SpeedRight, int ) {
  fl.spin(forward, SpeedLeft + SpeedRight, percent);
  flc.spin(forward, SpeedLeft + SpeedRight, percent);
  blc.spin(forward, SpeedLeft + SpeedRight, percent);
  bl.spin(forward, SpeedLeft + SpeedRight, percent);
  
  fr.spin(reverse, SpeedLeft - SpeedRight, percent);
  frc.spin(reverse, SpeedLeft - SpeedRight, percent);
  brc.spin(reverse, SpeedLeft - SpeedRight, percent);
  br.spin(forward, SpeedLeft + SpeedRight, percent);
  }

  void stopWheels () {
    fl.stop(brake);
    flc.stop(brake);
    blc.stop(brake);
    bl.stop(brake);
 
    fr.stop(brake);
    frc.stop(brake);
    brc.stop(brake);
    br.stop(brake);
   }

  //turn left
  void turnLeft(double angle) {
  // basically the same as right except left motor spins reverse and right is forward
  inertialSensor.setRotation(0, degrees);
  
  //turning left using inertial sensor
  while (fabs(inertialSensor.rotation(deg)) < angle) {
    double diff =  angle - fabs(inertialSensor.rotation(deg));
    // 5 + diff * 0.3 ,pct means to slow down when reaching the precent target.
    //You have to remember to set the minimum speed to 5 so it does not slowly move
    fl.spin(reverse, 5 + diff * 0.3, pct);
    flc.spin(reverse, 5 + diff * 0.3, pct);
    blc.spin(reverse, 5 + diff * 0.3, pct);
    bl.spin(forward, 5 + diff * 0.3, pct);
    
    fr.spin(forward, 5 + diff * 0.3, pct);
    frc.spin(forward, 5 + diff * 0.3, pct);
    brc.spin(forward, 5 + diff * 0.3, pct);
    br.spin(forward, 5 + diff * 0.3, pct);
   
    wait(5, msec);
  }
  stopWheels();
  }
  
  
  //turn right
  void turnRight(double angle) {
  // set inertial rotation to 0 degrees
  inertialSensor.setRotation(0, degrees);
  //turn right using inertial sensors
  while (inertialSensor.rotation(deg) < angle) {
    double diff =  angle - fabs(inertialSensor.rotation(deg));
    fl.spin(forward, 5 + diff * 0.3, pct);
    flc.spin(forward, 5 + diff * 0.3, pct);
    blc.spin(forward, 5 + diff * 0.3, pct);
    bl.spin(forward, 5 + diff * 0.3, pct);

    fr.spin(reverse, 5 + diff * 0.3, pct);
    frc.spin(reverse, 5 + diff * 0.3, pct);
    brc.spin(reverse, 5 + diff * 0.3, pct);
    br.spin(forward, 5 + diff * 0.3, pct);

    wait(5, msec);
  }
  stopWheels();
  }
  
  //set velocity
  void setVelocity(double vel) {
   // set all motors to velocity value of 'vel'
   fl.setVelocity(vel, percent);
   flc.setVelocity(vel, percent);
   blc.setVelocity(vel, percent);
   bl.setVelocity(vel, percent);

   fr.setVelocity(vel, percent);
   frc.setVelocity(vel, percent);
   brc.setVelocity(vel, percent);
   br.setVelocity(vel, percent);

  }
  //arcade
void arcade() {
  //Slower
  // int speedleft = controller1.Axis1.value()/2;
  // int speedright = controller1.Axis3.value()/2;
  // search up the ebot pilons tur`ning curves(or something like that) desmos
  
  double speedleft = controller1.Axis1.value() + controller1.Axis3.value();
  double speedright = controller1.Axis1.value() - controller1.Axis3.value();
  
  fl.spin(forward, speedleft, percent);
  flc.spin(forward, speedleft, percent);
  blc.spin(forward, speedleft, percent);
  bl.spin(forward, speedleft, percent);
  
  // RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
  fr.spin(reverse, speedright, percent);
  frc.spin(reverse, speedright, percent);  
  brc.spin(reverse, speedright, percent);
  br.spin(reverse, speedright, percent);
  }
  
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    arcade();
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
