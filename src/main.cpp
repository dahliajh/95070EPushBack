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

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


// PID
double kp = 0.175;
double ki = 0;
double kd = 0;

void pid(double targetDistance) {
 double error = targetDistance;
 double integral = 0;
 double lastError =  targetDistance;
 double prevDistanceError = fl.position(degrees);
 fl.setPosition(0, degrees);
 ml.setPosition(0, degrees);
 bl.setPosition(0, degrees);
 fr.setPosition(0, degrees);
 mr.setPosition(0, degrees);
 br.setPosition(0, degrees);
 while (true) {
   double measureDistance = (fl.position(degrees) + fr.position(degrees))/2;
   error = targetDistance - measureDistance;
   prevDistanceError = measureDistance;
   if (fabs(error)<30) {
     fl.stop(brake);
     ml.stop(brake);
     bl.stop(brake);

     fr.stop(brake);
     mr.stop(brake);
     br.stop(brake);
     return;
   }
   double speed = error * kp + integral * ki + (error - lastError) * kd;
   fl.spin(fwd, speed, percent);
   ml.spin(fwd, speed, percent);
   bl.spin(fwd, speed, percent);

   fr.spin(fwd, speed, percent);
   mr.spin(fwd, speed, percent);
   br.spin(fwd, speed, percent);

   lastError = error;
   wait(15, msec);
 }
}
// PID to inches
#define INCHES_TO_DEGREES 90/5
void pid_inches (double DistanceInInches) {
 double degrees = DistanceInInches * INCHES_TO_DEGREES;
 pid(degrees);
}

void moveAllWheels(int SpeedLeft, int SpeedRight, int ) {
  fl.spin(reverse, SpeedLeft + SpeedRight, percent);
  ml.spin(reverse, SpeedLeft + SpeedRight, percent);
  bl.spin(reverse, SpeedLeft + SpeedRight, percent);
  
  fr.spin(forward, SpeedLeft - SpeedRight, percent);
  mr.spin(forward, SpeedLeft - SpeedRight, percent);
  br.spin(forward, SpeedLeft + SpeedRight, percent);
  }

  void stopWheels () {
    fl.stop(brake);
    ml.stop(brake);
    bl.stop(brake);
 
    fr.stop(brake);
    mr.stop(brake);
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
    ml.spin(reverse, 5 + diff * 0.3, pct);
    bl.spin(forward, 5 + diff * 0.3, pct);
    
    fr.spin(forward, 5 + diff * 0.3, pct);
    mr.spin(forward, 5 + diff * 0.3, pct);
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
    ml.spin(forward, 5 + diff * 0.3, pct);
    bl.spin(forward, 5 + diff * 0.3, pct);

    fr.spin(reverse, 5 + diff * 0.3, pct);
    mr.spin(reverse, 5 + diff * 0.3, pct);
    br.spin(reverse, 5 + diff * 0.3, pct);

    wait(5, msec);
  }
  stopWheels();
  }
  
  //set velocity
  void setVelocity(double vel) {
   // set all motors to velocity value of 'vel'
   fl.setVelocity(vel, percent);
   ml.setVelocity(vel, percent);
   bl.setVelocity(vel, percent);

   fr.setVelocity(vel, percent);
   mr.setVelocity(vel, percent);
   br.setVelocity(vel, percent);

  }

  void simpletestauton () {
    pid_inches(35);
    turnLeft(87);
    wait(0.5, sec);
    stopWheels();
    pid_inches(25);
    turnRight(87);
    wait(0.5,sec);
    stopWheels();
    pid_inches(35);
    turnLeft(87);
    pid_inches(10);
    turnRight(87);
    pid_inches(40);
    turnLeft(170);
  }
  void blueright () { 
    controller1.Screen.print("Its a me mario");
  }
  
  int auton = 1;
//auton selector
void autonselector() {
  int numofautons = 2;
  if (controller1.ButtonX.pressing()) {
    auton++;
    wait(200,msec);
  } else if (controller1.ButtonB.pressing()) {
    auton--;
    wait(200,msec);
  }
  if (auton > numofautons) {
    auton = 1;
  } else if (auton < 1) {
    auton = numofautons;
  }
 
  if (auton == 1) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,9);
    controller1.Screen.print("Blue Right");
  } else if (auton == 2) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,4);
    controller1.Screen.print("Simple Test Auton");
  } 
 }
 
 // auton
 void autonomous(void) {
  controller1.Screen.print(" autons");
  if (auton == 1) {
    blueright();
  } else if (auton == 2){
    simpletestauton();
  } 
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

//arcade code
void arcade() {
  //Slower
  // int speedleft = controller1.Axis1.value()/2;
  // int speedright = controller1.Axis3.value()/2;
  // search up the ebot pilons tur`ning curves(or something like that) desmos
  
  double speedleft = controller1.Axis1.value() + controller1.Axis3.value();
  double speedright = controller1.Axis1.value() - controller1.Axis3.value();
  
  fl.spin(forward, speedleft, percent);
  ml.spin(forward, speedleft, percent);
  bl.spin(forward, speedleft, percent);
  
  // RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
  fr.spin(reverse, speedright, percent);
  mr.spin(reverse, speedright, percent);  
  br.spin(reverse, speedright, percent);
  }

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    arcade();
    wait(20, msec); // Sleep the task for a short amount of time to
  }
}

bool selecting = 1;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertialSensor.calibrate();
  wait(5, msec);
  waitUntil(!inertialSensor.isCalibrating());
  while (selecting) {
    autonselector();
   // if (controller1.ButtonB.pressing()) selecting = 0;
    wait(5, msec);
  }
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
