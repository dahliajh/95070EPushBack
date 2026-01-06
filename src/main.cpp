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

bool descore_up = false;
bool prevdescore_up = false;
bool matchload_down = false;
bool prevmatchload_down = false;
bool tpidflag = false;

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
double kp = 0.15; // tune this first
double ki = 0.15; // and lastly this
double kd = 0.13; // then this 
double pidTimeout = 3000; // 5-second timeout

void pid(double targetDistance) {
 double error = targetDistance;
 double integral = 0;
 double lastError =  targetDistance;
 vex::timer pidTimer;
 pidTimer.clear();
 double prevDistanceError = fl.position(degrees);
 fl.setPosition(0, degrees);
 fr.setPosition(0, degrees);
 ml.setPosition(0, degrees);
 mr.setPosition(0, degrees);
 bl.setPosition(0, degrees);
 br.setPosition(0, degrees);
 int stepCount = 0;
 while (true) {
   double measureDistance = (fl.position(degrees) + fr.position(degrees))/2;
   error = targetDistance - measureDistance;
   prevDistanceError = measureDistance;
   if (fabs(error)<30) {
     fl.stop(coast);
     fr.stop(coast);

     ml.stop(coast);
     mr.stop(coast);

     bl.stop(coast);
     br.stop(coast);
     //std::cout << " pid # of steps: " << stepCount << std::endl;
     return;
   }

   if (pidTimer.time(msec) > pidTimeout) {
     fl.stop(coast);
     fr.stop(coast);

     ml.stop(coast);
     mr.stop(coast);
     
     bl.stop(coast);
     br.stop(coast);
      std::cout << " timingout of pid after msec" << pidTimeout << std::endl;
      return;
    }

   double speed = error * kp + integral * ki + (error - lastError) * kd;
   //controller1.Screen.print("initial speed: %.2f ", speed);
   //std::cout<<"speed: " << speed<< " error: " << error << " p: " << error * kp << " d: " << (error - lastError) * kd << std::endl;
  
   stepCount = stepCount + 1;
  fl.spin(forward, speed, percent);
  fr.spin(forward, speed, percent);

  ml.spin(forward, speed, percent);
  mr.spin(forward, speed, percent);

  bl.spin(forward, speed, percent);
  br.spin(forward, speed, percent);

   lastError = error;
   wait(15, msec);
 }
 //std::cout << " pid # of steps: " << stepCount << std::endl;
}

// PID to inches
void pid_inches (double DistanceInInches) {
 double degrees = DistanceInInches * (4.0/3.0) * 360.0/(M_PI * 3.25);
 pid(degrees);
}

double tkp = 0.5; //tune this first
double tki = 0; //and lastly this
double tkd = 0.3; //then this

#include "vex.h"
#include <cmath>

void turnpid(double targetAngle) {

  double error = 0;
  double lastError = 0;
  double integral = 0;
  
  // Optional: Add a timeout
  vex::timer turnTimer;
  turnTimer.clear();
  double timeout = 3000; // 3-second timeout
  int stepCount = 0;
  while (true) {
    double measureAngle = inertialSensor.rotation(degrees);
    error = targetAngle - measureAngle;

    // Correctly handle the 180-degree wrap-around for the shortest path
    error = fmod(error + 180, 360) - 180;
    
    // std::cout << "error: " << error <<std::endl;
    if (fabs(error) < 4.0) { // Reduced tolerance for better accuracy
      fl.stop(brake);
      ml.stop(brake);
      bl.stop(brake);

      fr.stop(brake);
      mr.stop(brake);
      br.stop(brake);
      std::cout << " turnpid # of steps: " << stepCount << std::endl;
      return; 
    }
    
    if (turnTimer.time(msec) > timeout) {
      fl.stop(coast);
      ml.stop(coast);
      bl.stop(coast);

      fr.stop(coast);
      mr.stop(coast);
      br.stop(coast);
      std::cout << " turnpid # of steps: " << stepCount << std::endl;
      return;
    }
    
    //Accumulate error over time
    integral += error;

    if (fabs(error) > 5) { // Changed this to a smaller value for anti-windup
      integral = 0; // Reset integral if the error is large
    }

    double derivative = error - lastError;
    double speed = (error * tkp) + (integral * tki) + (derivative * tkd);
    // controller1.Screen.print("turn speed: %.2f ", speed);

    //std::cout<<"turn sp: " << speed <<std::endl;
    if (tpidflag) {
      std::cout<<"turnspeed: " << speed << " error: " << error << " p: " << error * tkp << " d: " << (error - lastError) * tkd << std::endl;
    }
    stepCount = stepCount + 1;

    // Clamp the speed to prevent it from going over 100%
    if (fabs(speed) > 100) {
        speed = 100 * sin(speed);
    }
    // Controller1.Screen.print
    // Spin the motors with adjusted directions based on the error
    fl.spin(fwd, speed, percent);
    ml.spin(fwd, speed, percent);
    bl.spin(fwd, speed, percent);

    fr.spin(fwd, -speed, percent); // Note the negative sign here
    mr.spin(fwd, -speed, percent);
    br.spin(fwd, -speed, percent);
 
    lastError = error;
    wait(15, msec);
    //std::cout<<"err: " << error<<std::endl;
    //std::cout<<"sensor: " << inertialSensor.rotation(degrees)<<std::endl;
  }
  std::cout << " turnpid # of steps: " << stepCount << std::endl;

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

void intaking() { /// PUT ALL SCORING IN THE SAME FUNCTION
// hold blocks
  if (controller1.ButtonL1.pressing()) { 
    intake.spin(forward, 95, pct);
  }
//score bottom/outtake
    else if (controller1.ButtonL2.pressing()) {
    intake.spin(reverse, 95, pct);
    intake2.spin(reverse, 95, pct);
    //intake3.spin(reverse, 85, pct);
  }
// score middle goal
    else if (controller1.ButtonR1.pressing()) {  
    intake.spin(forward, 95, pct);
    intake2.spin(reverse, 95, pct);

  }
  
//score high
  else if (controller1.ButtonR2.pressing()) { 
    intake.spin(forward, 95, pct);
    intake2.spin(forward, 95, pct);
  
  }
 
    else {
    intake.stop(brake);
    intake2.stop(brake);
    //intake3.stop(brake);
  }
}

void descore_up_fn () {
  descore.set(false);
  std::cout << " descore_up " << std::endl;
}
void descore_down_fn () {
  descore.set(true);
  std::cout << " descore_down " << std::endl;
}

void matchload_down_fn () {
  matchloader.set(true);
}

void matchload_up_fn () {
  matchloader.set(false);
}

void runIntake () {
  intake.spin(forward, 95, pct);
  intake2.stop(coast);
}

void runBasket () {
  intake.spin(forward, 95, pct);
  //intake3.spin(forward, 90, pct);
}

void stopIntake () {
  intake.stop(coast);
  //intake3.stop(coast);
}

void runtopintake () {
  intake.spin(forward, 95, pct);
  intake2.spin(forward, 95, pct);
}

void runoutake () {
  intake.spin(reverse, 95, pct);
  intake2.spin(reverse, 95, pct);
}
void runmiddletop () {
  intake.spin(forward, 95, pct);
  intake2.spin(reverse, 95, pct);
  //intake3.spin(reverse, 90, pct);
}

void stoptopintake () {
  intake2.stop(coast);
}

void downmatchload () {
  matchloader.set(true);
}

void upmatchload () {
  matchloader.set(false);
}

void descoreup () {
  descore.set(true);
}

void descoredown () {
  descore.set(false);
}
void simpletestauton () {
  kp = 0.05;
  pid_inches(30);
  wait(1, sec);
  pid_inches(-30);
}

void rightside () { 
  std::cout<<"RIGHT SIDE" <<std::endl;
  kp = 0.057;
  tkp = 0.5;
  runIntake();
  pid_inches(20);
  stopIntake();
  //go to top goal
  kp = 0.052;
  pid_inches(-10);
  //turn straight ahead
  tkp = 0.5;
  turnpid(78);
  kp = 0.052;
  pid_inches(33);
  //turn into long goal and score
  //tkd = 0.4;
  //tkp = 0.75;
  tkd = 0.5;
  turnpid(348);
  kp = 0.052;
  runtopintake();
  pid_inches(12);
  wait(1.1, sec);
  //middle goal
  kp = 0.052;
  pid_inches(-12);
  //tkp = 1.5;
  //turn in direction of middle goal
  turnpid(302);
  runIntake();
  kp = 0.06;
  pid_inches(25);
  //score in middle goal
  kp = 0.067;
  pid_inches(22);
  runoutake();
}

void rightside4blocknew () {
  std::cout << "Right side new" << std::endl;
  kp = 0.057;
  tkp = 0.3;
  runIntake();
  //1st and 2nd block
  pid_inches(20);
  //wait(0.07, sec);
  //3rd block
  kp = 0.05;
  pid_inches(3);
  wait(0.05, sec);
  pid_inches(2);
  wait(0.05, sec);
  //go to long goal 
  turnpid(105);
  pid_inches(30.5);
  // tkp = 0.7;
  tpidflag = true;
  turnpid(163);
  tpidflag = false;
  kp = 0.06;
  pid_inches(-15);
  pid_inches(2);
  runoutake();
  wait(0.2, sec);
  runtopintake();
  wait(2.5, sec);
  //pid_inches(20);
}

void rightsidedescore () {
  kp = 0.09;
  kd = 0.1;
  runIntake();
  pid_inches(10);
  turnpid(200);
  pid_inches(-13);
  turnpid(163);
  descoreup();
  descoredown();
  pid_inches(-16);
}
void rightside4push () {
  kp = 0.057;
  tkp = 0.3;
  runIntake();
  //1st and 2nd block
  pid_inches(20);
  //3rd block
  kp = 0.05;
  pid_inches(3);
  wait(0.05, sec);
  pid_inches(2);
  wait(0.07, sec);
  //go to long goal 
  turnpid(90);
  pid_inches(28.5);
  //tpidflag = true;
  turnpid(163);
  //tpidflag = false;
  //kp = 0.045;
  //kd = 0.2;
  pidTimeout = 1000;
  pid_inches(-7);
  //std::cout << "10 inches" << std::endl;
  // pid_inches(-6.5);
  //std::cout << "before long goal outtake" << std::endl;
  //pid_inches(2);
  // runoutake();
  // wait(0.2, sec);
  runtopintake();
  wait(2, sec);
  //descore part
  rightsidedescore();
  //my idea for fixing getting the blocks stuck: 
  //create something that says if the intake doesn't give you an output, 
  //it will have to run the outtake function and then rerun the intake function 
  // and we use the timeout to make sure that it doesnt keep doing this for too long
}

void rightside4middle () {
  kp = 0.057;
  tkp = 0.3;
  runBasket();
  //1st and 2nd block
  pid_inches(20);
  //wait(0.07, sec);
  //3rd block
  runBasket();
  kp = 0.05;
  pid_inches(3);
  wait(0.05, sec);
  runBasket();
  pid_inches(2);
  wait(0.05, sec);
  runBasket();
  pid_inches(5);
  runBasket();
  // go to low middle goal
  turnpid(294);
  pid_inches(13.3);
  runoutake();
  // pid_inches(-2);
  // pid_inches(2);
  // pid_inches(-2);
  // pid_inches(2);
}
void rightside4blockold () {
  kp = 0.057;
  tkp = 0.4;
  runIntake();
  //1st block
  pid_inches(18);
  wait(0.07, sec);
  pid_inches(-2);
  //2nd block
  turnpid(7);
  pid_inches(3);
  wait(0.07, sec);
  //3rd block
  turnpid(-3);
  pid_inches(5);
  wait(0.07, sec);
  //go to long goal
  tkp = 0.35;
  turnpid(105);
  pid_inches(33);
  turnpid(161);
  pid_inches(-13);
  runtopintake();
}

void leftside () {
  kp = 0.052;
  tkp = 0.5;
  runIntake();
  pid_inches(22);
  stopIntake();
  pid_inches(-10);
  turnpid(287);
  pid_inches(32);
  tkd = 0.4;
  turnpid(13);
  runtopintake();
  pid_inches(7);
  wait(1, sec);
}

void leftsidedescore () {
  pid_inches(10);
  turnpid(210);
  pid_inches(-15);
  pid_inches(190);
}

void leftside7block () {
  kp = 0.057;
  tkp = 0.3;
  runIntake();
  //1st and 2nd block
  kp = 0.05;
  pid_inches(19.5);
  //3rd block
  pid_inches(3);
  //wait(0.05, sec);
  pid_inches(2);
  //wait(0.05, sec);
  //go to long goal 
  turnpid(262);
  pid_inches(32);
  //tpidflag = true;
  tkp = 0.25;
  turnpid(210);
  //tpidflag = false;
  pidTimeout = 1000;
  pid_inches(-15);
  //wait(0.02, sec);
  runtopintake();
  wait(1, sec);
  // force alignment to straighten the bot before going to matchload
  pid_inches(2);
  pidTimeout = 3000;
  // matchload
  kp = 0.067;
  downmatchload();
  runIntake();
  pid_inches(20);
  turnpid(200);
  pid_inches(6.7);
  wait(0.567, sec);
  kp = 0.05;
  pid_inches(-26);
  turnpid(210);
  runtopintake();
}
void leftsidenew () {
  kp = 0.057;
  tkp = 0.3;
  runIntake();
  //1st and 2nd block
  pid_inches(19.5);
  //3rd block
  kp = 0.05;
  pid_inches(3);
  wait(0.05, sec);
  pid_inches(2);
  wait(0.05, sec);
  //go to long goal 
  turnpid(262);
  pid_inches(30.5);
  //tpidflag = true;
  tkp = 0.25;
  turnpid(190);
  //tpidflag = false;
  //kp = 0.057;
  pid_inches(-14);
  wait(0.02, sec);
  // runoutake();
  // wait(0.2, sec);
  runtopintake();
  wait(2.5, sec);
  //pid_inches(20);
}

void skillsauton () {
  kd = 0.1;
  kp = 0.027;
  tkp = 0.50;
//get balls from matchloads
  downmatchload();
  pid_inches(34);
  turnpid(70);
  // wait(1, sec);
  runIntake();
  kp = 0.06;
  pid_inches(9);
  wait(4, sec);
  pid_inches(-5);
  wait(100, msec);
  pid_inches(6);
  wait(4, sec);
  pid_inches(-5);
  kp = 0.027;
  //pid_inches(7.5);
  //wait(40, msec);
//score in long goal
  pid_inches(-22);
  turnpid(95);
  pid_inches(-2);
  //pid_inches(8);
  //turnpid(90);
  ///kp = 0.06;
  //pid_inches(-8);
  //kp = 0.027;
  runoutake();
  wait(0.3, sec);      
  runtopintake();
  wait(4, sec);
//collect 4 balls and score
  upmatchload();
  stopIntake();
  pid_inches(17);
  tkp = 0.2;
  turnpid(222);
  upmatchload();
  runIntake();
  pid_inches(27);
  wait(1, sec);
  pid_inches(20);
  runoutake();
  wait(2, sec);
////Park
  pid_inches(-21);
  turnpid(270);
  //runBasket();
  pid_inches(-33);
  turnpid(170);
  //runBasket();
  // wait(20, msec);
  kp = 0.075;
  pid_inches(53);
  // wait(1, sec);
  // pid_inches(-5); 
}

void oldskillsauton () {
  kd = 0.1;
  kp = 0.027;
  tkp = 0.5;
  //get balls from matchloads
  downmatchload();
  pid_inches(34);
  turnpid(78.5);
  // wait(1, sec);
  runIntake();
  kd = 0.02;
  pid_inches(10);
  wait(4, sec);
  pid_inches(-5);
  wait(500, msec);
  pid_inches(9);
  wait(4, sec);
  pid_inches(-5);
  kd = 0.07;
  //wait(30, msec);
  //pid_inches(7.5);
  //wait(40, msec);
  //score in long goal
  pid_inches(-22);
  //pid_inches(8);
  //turnpid(90);
  ///kp = 0.06;
  //pid_inches(-8);
  //kp = 0.027;
  runoutake();
  wait(300, msec);      
  runtopintake();
  wait(6, sec);
  //collect 4 balls and score
  upmatchload();
  stopIntake();
  pid_inches(19);
  tkp = 0.2;
  turnpid(222);
  upmatchload();
  runIntake();
  pid_inches(26);
  wait(1, sec);
  pid_inches(21);
  //turnpid(222);
  //turnpid(240);
  runoutake();
  wait(2, sec);
  //Step 3!!
  pid_inches(-22);
  turnpid(180);
  pid_inches(50);
  turnpid(75);
  pid_inches(32);
  turnpid(180);
  pid_inches(22);
  turnpid(77);
  /*downmatchload();
  runIntake();
  pid_inches(7);
  wait(4, sec);
  pid_inches(-27);
  stopIntake();
  runoutake();
  wait(500, msec);
  runtopintake();*/
}

void skillsautonpark() {
  kp = 0.1;
  kd = 0.05;
  runIntake();
  pid_inches(-15);
  pid_inches(45);
  wait(1.5, sec);
  pid_inches(-10);
  pid_inches(10);
}  

int auton = 1;
//auton selector
void autonselector() {
  int numofautons = 12;
  if (controller1.ButtonRight.pressing()) {
    auton++;
    wait(200,msec);
  } else if (controller1.ButtonLeft.pressing()) {
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
    controller1.Screen.print("Right Side");
  } else if (auton == 3) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,4);
    controller1.Screen.print(" Right Side 4 Block");
  } else if (auton == 4) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,5);
    controller1.Screen.print("Right Side 4 Push");
  } else if (auton == 5) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,6);
    controller1.Screen.print("Right Side 4BS");
  } else if (auton == 6) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Left Side");
  } else if (auton == 7) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,6);
    controller1.Screen.print("Left Side New");
  } else if (auton == 8) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Left Side 7");
  } else if (auton == 9) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,12);
    controller1.Screen.print("Skills");
  } else if (auton == 10) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,10);
    controller1.Screen.print("Old Skills");
  } else if (auton == 11) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,6);
    controller1.Screen.print("rightside4middle");
  } else if (auton == 12) {
    controller1.Screen.clearScreen();
    controller1.Screen.setCursor(2,8);
    controller1.Screen.print("Skills Park");
  }

}

 // auton
void autonomous(void) {
  if (auton == 1) {
    simpletestauton();
  } else if (auton == 2){
    rightside();
  } else if (auton == 3){
    rightside4blocknew();
  } else if (auton == 4){
    rightside4push();
  } else if (auton == 5){
    rightside4blockold();
  } else if (auton == 6){
    leftside();
  } else if (auton == 7){
    leftsidenew();
  } else if (auton == 8){
    leftside7block();
  } else if (auton == 9){
    skillsauton();
  } else if (auton == 10){
    oldskillsauton();
  } else if (auton == 11){
    rightside4middle();
  } else if (auton == 12){
    skillsautonpark();
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

  // double speedleft = controller1.Axis1.value() * 0.75 + controller1.Axis3.value() * 0.75;
  // double speedright = controller1.Axis1.value() * 0.75 - controller1.Axis3.value() * 0.75;
  
  double axis1_value = controller1.Axis1.value();
   if (controller1.Axis1.value() > -10 && controller1.Axis1.value() < 10)
  { axis1_value = 0; }

  double axis3_value = controller1.Axis3.value();
   if (controller1.Axis3.value() > -10 && controller1.Axis3.value() < 10) 
   {axis3_value = 0;}

  double speedleft = axis1_value * 0.75 + axis3_value * 0.75;
  double speedright = axis1_value * 0.75 - axis3_value * 0.75;
  
  // LEFT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!! 
  fl.spin(forward, speedleft, percent);
  ml.spin(forward, speedleft, percent);
  bl.spin(forward, speedleft, percent);
  
  fr.spin(reverse, speedright, percent);
  mr.spin(reverse, speedright, percent);  
  br.spin(reverse, speedright, percent);
}

// void usercontrol(void) {
//   // User control code here, inside the loop
//   while (1) {
//     arcade();
//     intaking();
//   // descore
//   if (controller1.ButtonY.pressing()) {
//       // Only toggle if we haven't already registered this specific press
//       if (!prevdescore_up) {
//         descore_up = !descore_up;
//         descore.set(descore_up);
//         prevdescore_up = true;
//       }
//     } 
//     else {
//       // ONLY reset the latch when the button is RELEASED
//       prevdescore_up = false;
//     }
//   //  if (controller1.ButtonY.pressing()) {
//   //   if (prevdescore_up == false) {
//   //     descore_up = !descore_up;
//   //     prevdescore_up = true;
//   //   } else {
//   //     if (prevdescore_up == true) {
//   //       prevdescore_up = false;
//   //     }
//   //   } descore.set(descore_up);
//     // controller1.ButtonY.released(descore_down_fn);
//     wait(20, msec); 
//   }
 

//  //matchloader
//   if (controller1.ButtonX.pressing()) {  
//    if (!prevmatchload_down) {
//     matchload_down = !matchload_down;
//     matchloader.set(matchload_down);
//     prevmatchload_down = true; 
//     }
//   } else {
//    prevmatchload_down = false;
//  } wait(20, msec); 
// }
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    arcade();
    intaking();
    
    // descore
    if (controller1.ButtonY.pressing()) {
      // Only toggle if we haven't already registered this specific press
      if (!prevdescore_up) {
        descore_up = !descore_up;
        descore.set(descore_up);
        prevdescore_up = true;
      }
    } 
    else {
      // ONLY reset the latch when the button is RELEASED
      prevdescore_up = false;
    }
    
    // The problematic wait(20, msec); was here. It has been removed.
    
    // matchloader
    if (controller1.ButtonX.pressing()) {  
      if (!prevmatchload_down) {
        matchload_down = !matchload_down; 
        matchloader.set(matchload_down);
        prevmatchload_down = true; 
      }
    } else {
      prevmatchload_down = false;
    } 
    
    // Place the single wait at the very end of the while(1) loop
    wait(20, msec); 
  }
}
 
bool selecting = 1;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertialSensor.setRotation(0, degrees);
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


//void kathy () {
//KL.spin(reverse, 67.41, pct);
//F}