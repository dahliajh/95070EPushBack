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
#include "robot-config.h"
#include <iostream>

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
   fl.spin(forward, speed, percent);
   ml.spin(forward, speed, percent);
   bl.spin(forward, speed, percent);

   fr.spin(forward, speed, percent);
   mr.spin(forward, speed, percent);
   br.spin(forward, speed, percent);

   lastError = error;
   wait(15, msec);
 }
}

// PID to inches
void pid_inches (double DistanceInInches) {
 double degrees = DistanceInInches * (4.0/3.0) * 360.0/(M_PI * 3.25);
 pid(degrees);
}

double tkp = 0.5;
double tki = 0.7;
double tkd = 0.5;

//turn pid
void turnpid (double targetAngle) {
  double error = targetAngle;
  double integral = 0;
  double lastError =  targetAngle;
  double prevDistanceError = fl.position(degrees);
  inertialSensor.setRotation(0, degrees);
  while (true) {
    double measureAngle = inertialSensor.rotation(degrees);
    error = targetAngle - measureAngle;
    prevDistanceError = measureAngle;
    if (fabs(error)<3) {
      fl.stop(brake);
      ml.stop(brake);
      bl.stop(brake);
 
      fr.stop(brake);
      mr.stop(brake);
      br.stop(brake);
      return;
    }
    double speed = error * kp + integral * ki + (error - lastError) * kd;
    fl.spin(reverse, speed, percent);
    ml.spin(reverse, speed, percent);
    bl.spin(reverse, speed, percent);
 
    fr.spin(forward, speed, percent);
    mr.spin(forward, speed, percent);
    br.spin(forward, speed, percent);
 
    lastError = error;
    wait(20, msec);
    std::cout<<"err: " << error<<std::endl;
    std::cout<<"sensor: " << inertialSensor.rotation(degrees)<<std::endl;
  }
}

void moveAllWheels(int SpeedLeft, int SpeedRight) {
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
// intaking bottom and middle and outtaking inside intake
void intaking() {
  if (controller1.ButtonR1.pressing()) {
  intake.spin(forward, 85, pct);
  intake3.spin(reverse, 85, pct);

  } else if (controller1.ButtonR2.pressing()) {
  intake.spin(reverse, 85, pct);
  intake3.spin(forward, 85, pct);

  } else {
  intake.stop(coast);
  intake3.stop(coast);
  }
}   
//intaking top intake
void intaking2 () {
  if (controller1.ButtonL1.pressing()) {
    intake2.spin(forward, 85, pct);

  } else if (controller1.ButtonL2.pressing()) {
    intake2.spin(reverse, 85, pct);

  } else {
    intake2.stop(coast);
  }
}

void runIntake () {
  intake.spin(forward, 95, pct);
  intake3.spin(reverse, 90, pct);
}

void runOutake () {
  intake.spin(reverse, 90, pct);
  intake3.spin(forward, 90, pct);
}

void stopIntake () {
  intake.stop(coast);
  intake3.stop(coast);
}

void runtopintake () {
  intake.spin(forward, 95, pct);
  intake3.spin(reverse, 90, pct);
  intake2.spin(forward, 95, pct);
}
void runtopoutake () {
  intake2.spin(reverse, 90, pct);
}

void stopintake2 () {
  intake2.stop(coast);
}

void simpletestauton () {
  pid(500);
}
void blueright () { 

}

void blueleft () {

}

void redright () {
  kp = 0.15;
  pid_inches(-33);
  wait(500, msec);
  turnpid(-47);
  wait(0.1, sec);
  kp = 0.05;
  pid_inches(-3);
  runtopintake();
}

void redleft () {
  pid_inches(40);
  turnpid(90);
}

 
int auton = 1;
//auton selector
void autonselector() {
  int numofautons = 5;
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
    controller1.Screen.setCursor(2,4);
    controller1.Screen.print("Simple Test Auton");
  } else if (auton == 2) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,9);
    controller1.Screen.print("Blue Right");
  } else if (auton == 3) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Blue Left");
  } else if (auton == 4) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Red Right");
  } else if (auton == 5) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Red Left");
  }
}
 
 // auton
void autonomous(void) {
  if (auton == 1) {
    simpletestauton();
  } else if (auton == 2){
    blueright();
  } else if (auton == 3){
    blueleft();
  } else if (auton == 4){
    redright();
  } else if (auton == 5){
    redleft();
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

  double speedleft = controller1.Axis1.value() * 0.75 + controller1.Axis3.value() * 0.75;
  double speedright = controller1.Axis1.value() * 0.75 - controller1.Axis3.value() * 0.75;
  
  // LEFT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!! 
  fl.spin(reverse, speedleft, percent);
  ml.spin(reverse, speedleft, percent);
  bl.spin(reverse, speedleft, percent);
  
  fr.spin(forward, speedright, percent);
  mr.spin(forward, speedright, percent);  
  br.spin(forward, speedright, percent);
  }

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    arcade();
    intaking();
    intaking2();
    wait(20, msec); 
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
    wait(5, msec);
  }
  // All activities that occur before the competition starts
}

// Main will set up the competition functions and callbacks.

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
