#pragma once

#include <iostream>
#include <string>
#include "Robot.h"

#include <frc/TimedRobot.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/smartdashboard/SendableChooser.h>

#include "ctre/Phoenix.h"
#include <rev/CANSparkMax.h>
#include <math.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

//microswitch imports
#include "frc/DigitalInput.h"


//limelight stuff
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

//gyro
#include <frc/ADXRS450_Gyro.h>

using namespace std;

//timers for autonomous
frc::Timer *sTimer {nullptr};
frc::Timer *aTimer {nullptr};

//Finds distance of robot to reflective tape on hub using limelight
double Robot::LimelightDistance() {
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto limelight = inst.GetTable("limelight");

    targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
    targetOffsetAngle_Vertical = limelight->GetNumber("ty",0.0);
    targetArea = limelight->GetNumber("ta",0.0);
    targetSkew = limelight->GetNumber("ts",0.0);
    
  
  // Distance = (targetHeight - cameraHeight)/tan(mountingAngle + verticalAngleToTarget) + 
  // (distance from rim to center of hub + distance from limelight to the center of where the ball leaves)

    trueAngle = (39 + targetOffsetAngle_Vertical) * M_PI / 180;
    distanceToHub = (104-23.8)/(tan(trueAngle)) + (24 + 6.868);

    frc::SmartDashboard::PutNumber("Distance: ", distanceToHub);
    return distanceToHub;
}

//the distance can be a positive or a negative
//input a negative speed to go backwards, positive to go forward
void Robot::drive(double distance, double speed) {
  gyroAngle = gyro.GetAngle();

  //Converts distance in inches to counts for drive motors
  targetDistance = 0.7317 * (distance * countsPerInch);

  averageActualDistance = (-(leftEncoder.GetPosition()) + rightEncoder.GetPosition())/2;

  //Determines distance where the robot will slow down 
  fractionOfTargetDistance = targetDistance * (1.0/3); 

  //Variable to adjust robot speed based on drift
  speedFactor = error * 0.1; 

  //Adjusts left and right motor speeds to account for drift measured by the gyro sensor
  if(abs(leftEncoder.GetPosition()) < abs(targetDistance) && abs(rightEncoder.GetPosition()) < abs(targetDistance)){
    if(error < -1){
      rightFront.Set(-(speed + (speed * speedFactor)));
      leftFront.Set(speed);
    } 
    else if (error > 1) {
      leftFront.Set(speed + (speed * speedFactor));
      rightFront.Set(-speed);      
    } 
    else {
      rightFront.Set(-speed);
      leftFront.Set(speed);   
    } 

    //slows robot down when it reaches a fraction of target distance 
    if(abs(leftEncoder.GetPosition()) >= abs(fractionOfTargetDistance) && abs(rightEncoder.GetPosition()) >= abs(fractionOfTargetDistance)) {
      speed /= 8;      
    } 

  } 
  else if(abs(leftEncoder.GetPosition()) < abs(targetDistance) && abs(rightEncoder.GetPosition()) < abs(targetDistance)) {
    if (error > 1) {
      rightFront.Set(-(speed + (speed * speedFactor)));
      leftFront.Set(speed);
    } 
    else if (error < -1) {
      leftFront.Set((speed + (speed * speedFactor)));
      rightFront.Set(-speed); 
        
    }
    else {
      rightFront.Set(-speed);
      leftFront.Set(speed); 
    }

    if (abs(leftEncoder.GetPosition()) <= abs(fractionOfTargetDistance) && abs(rightEncoder.GetPosition()) <= abs(fractionOfTargetDistance)) {
      speed /= 8;     
    }

  } 
  else {
    leftFront.Set(0);
    rightFront.Set(0);

    //reset timer if motors are not moving
    encoderDriveRunning = false;
    sTimer->Reset();
    aTimer->Reset();

    frc::SmartDashboard::PutNumber("Left Encoder value", -leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Right Encoder value", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("target", targetDistance);
  }
}  

//alligns robot with hub using limelight 
void Robot::limelightAlign() {
  //remakes the limelight data table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto limelight = inst.GetTable("limelight");
  
  //needs to be configured 
  float constant = 0.01; 
  double leftChange, rightChange;

  targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
  frc::SmartDashboard::PutNumber("horizontal offset", targetOffsetAngle_Horizontal);

  //59.6 horizontal degrees FOV
  //If buton x is pressed...
  if(gamepad2.GetRawButton(4)) { 
    //if angle is geater than 3 degrees, adjust robot to align with hub 
    if(targetOffsetAngle_Horizontal > 3) {
      leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
      rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

      leftFront.Set(leftChange);
      rightFront.Set(rightChange);
    } 
    //if angle is less than -3 degrees, adjust robot to align with hub 
    else if(targetOffsetAngle_Horizontal < -3) {
      leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
      rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

      leftFront.Set(-leftChange);
      rightFront.Set(-rightChange);
    }
  }
  //if button x is not pressed, set motor speeds to 0 
  else if(!(rightDriveRunning && leftDriveRunning)) { 
    leftFront.Set(0);
    rightFront.Set(0);
  }
}

//called in auto to make robot align with hub using limelight
void Robot::autoLimelightAlign() {
  
  auto alignTime = aTimer->Get();  
  alignElapsedTime += (alignTime - alignPreviousTime);
  
  if(alignElapsedTime >= units::second_t(2)) {
      auto inst = nt::NetworkTableInstance::GetDefault();
      auto limelight = inst.GetTable("limelight");
      
      //needs to be configured 
      float constant = 0.01; 
      float min_command = 0.05f;
      double leftChange, rightChange;

      //current speed of the motors
      double leftSpeed = leftFront.GetAppliedOutput();
      double rightSpeed = rightFront.GetAppliedOutput();

      //if angle is geater than 3 degrees, adjust robot to allign with hub 
      if(targetOffsetAngle_Horizontal > 3) {
        leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
        rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

        leftFront.Set(leftChange);
        rightFront.Set(rightChange);
      } 
      //if angle is less than -3 degrees, adjust robot to allign with hub 
      else if(targetOffsetAngle_Horizontal < -3) {
        leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
        rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

        leftFront.Set(-leftChange);
        rightFront.Set(-rightChange);
      } 
      else {
        leftFront.Set(0);
        rightFront.Set(0);
        alignmentComplete = true;
      }
    alignPreviousTime = alignTime; 
  }
}

//used to shoot cargo 
void Robot::shoot(){
  
  //makes motors stop
  leftFront.Set(0);
  rightFront.Set(0);
 
  //calling method to change distance recieved from limelight to rpm value 
 
  flywheelPID.SetReference(-DistanceToRPM(LimelightDistance()), rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("MotorVelocity", motorVelocity);
  
  
  auto time = sTimer->Get();  
  
  elapsedTime += (time - previousTime);

  //if at or past 5 seconds, set all motor speeds to 0 
  if (elapsedTime >= units::second_t(7)) {
    intake.Set(0);
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);
    hConveyor.Set(0);

    flywheelPID.SetReference(0, rev::ControlType::kVelocity);

    functionCompleted = 1;
    //delete sTimer;
  } //if at or past 3 seconds, turn on motors for intake and conveyors
  else if (elapsedTime >= units::second_t(3)) {
    intake.Set(-1);
    vConveyorLeft.Set(-1);
    vConveyorRight.Set(1);
    hConveyor.Set(-1);
  }
  
  previousTime = time; 
}
  
//shoots cargo in lower hub 
void Robot::lowerPortShot() {    
    //NOTE: Setting target velocity = setting target RPM
    isActive = true;

    hConveyor.Set(1);
    //DistanceToRPM(LimelightDistance());
    flywheelPID.SetReference(-1300, rev::ControlType::kVelocity);
    frc::Wait(units::second_t(1));

    vConveyorLeft.Set(1);
    vConveyorRight.Set(-1);
    
    //after 4 seconds, set all motor speeds to 0 
    frc::Wait(units::second_t(4));
    
    intake.Set(0);
    hConveyor.Set(0);
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);

    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
    
    isActive = false;
    
  frc::SmartDashboard::PutNumber("Set rpm", motorVelocity);
  frc::SmartDashboard::PutNumber("True rpm", trueVelocity);
}

// takes distance (in inches) and converts it to a rpm using
//changes distance value from limelight to rpm
double Robot::DistanceToRPM (double distance) {
  return 2 * (int)(8.56*(distance+10) + (1097+50));
}

//returns varrying integers based on location of cargo for automated intake function in tele-op
int Robot::getPosition() {
    //if both switches sense cargo, return 1 
    if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 1){
      return 1;
    }
    
    //if horizontal switch senses cargo, but vertical switch does not sense cargo, return 2
    if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 0){
        return 2;
    }  
    
    //if horizontal switch does not sense cargo, but vertical switch does sense cargo, return 3 
    if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 1){
        return 3;
     } 
     
    //if both horizontal and vertical switch do not sense cargo, return 4 
     if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 0){
        return 4;

    } 
}
//turns on/off motors to intake cargo depending on microswitch data
void Robot::microswitchIntake(){
    frc::SmartDashboard::PutNumber("GetPosition function", getPosition());
    if (getPosition() == 1){
       //if both switches sense cargo...
      hConveyor.Set(0);
      intake.Set(0);
      vConveyorLeft.Set(0);
      vConveyorRight.Set(0);
       //conveyor motors will not turn on because robot already possesses two cargo
    } 
    else if (getPosition() == 2){
      //if horizontal switch senses cargo and vertical switch does not sense cargo...
      if (verticalSwitch.Get() == 0){
        vConveyorLeft.Set(1);
        vConveyorRight.Set(-1);
        hConveyor.Set(1);
    } //vertical conveyor will turn on until cargo moves to vertical sensor position
        if (horizontalSwitch.Get() == 0){
          hConveyor.Set(1);
          intake.Set(1);
        } //horizontal conveyor will turn on until cargo moves to horizontal sensor position
    }
    else if (getPosition() == 3){
      //if horizontal switch does not sense cargo and vertical switch senses cargo...
      if (horizontalSwitch.Get() == 0){
        hConveyor.Set(1);
        intake.Set(1);
      } //horizontal conveyor will turn on until cargo moves to horizontal sensor position
    } 
    else if (getPosition() == 4){
      //if both horizontal and vertical switch do not sense cargo 
      if (verticalSwitch.Get() == 0){
        vConveyorLeft.Set(1);
        vConveyorRight.Set(-1);
      } //vertical conveyor will turn on until cargo moves to vertical sensor position
      if (horizontalSwitch.Get() == 0){
        hConveyor.Set(-1);
        intake.Set(-1);
      } //horizontal conveyor will turn on until cargo moves to horizontal sensor position
    } 
    else {
      vConveyorLeft.Set(0);
      vConveyorRight.Set(0);
      hConveyor.Set(0);
      intake.Set(0);
     }
}

void::Robot::tankDrive(){
  //left motors
  
  if(!isShooting) {
    if(Attack32.GetRawButton(1) && (Attack31.GetRawAxis(1) >= 0.1 || Attack31.GetRawAxis(1) <= -0.1)) {
      leftFront.Set(-Attack31.GetRawAxis(1) * 0.35);
      frc::SmartDashboard::PutBoolean("Left Running:", true);

    }
    else if(Attack31.GetRawAxis(1) >= 0.1 || Attack31.GetRawAxis(1) <= -0.1) {
      leftFront.Set(-Attack31.GetRawAxis(1) * 0.7);
      frc::SmartDashboard::PutBoolean("Left Running:", true);
    }
    else {
      leftFront.Set(0);
      frc::SmartDashboard::PutBoolean("Left Running:", false);
      leftDriveRunning = false;
    }   
    
    //Right motors
    if(Attack32.GetRawButton(1) && (Attack32.GetRawAxis(1) >= 0.1 || Attack32.GetRawAxis(1) <= -0.1)) {
      rightFront.Set(Attack32.GetRawAxis(1) * 0.35);
      frc::SmartDashboard::PutBoolean("Right Running:", true);

    }
    else if(Attack32.GetRawAxis(1) >= 0.1 || Attack32.GetRawAxis(1) <= -0.1) {
      rightFront.Set(Attack32.GetRawAxis(1) * 0.7);
      frc::SmartDashboard::PutBoolean("Right Running:", true);

    }
    else {
      rightFront.Set(0);
      frc::SmartDashboard::PutBoolean("Right Running:", false);
      rightDriveRunning = false;
    }   
  }
}

//Kent Preference
void::Robot::kentController(){

  //when right bumper is pressed, turn on intake and horizontal conveyor
  if(gamepad2.GetRawButtonPressed(6)){
    intake.Set(1);
    hConveyor.Set(1);
  } //when left bumper is pressed, reverse intake and horizontal conveyor
  else if(gamepad2.GetRawButtonPressed(5)){ 
    intake.Set(-1);
    hConveyor.Set(-1);
  } //turn off intake and horizontal conveyor otherwise
  else {
    intake.Set(0);
    hConveyor.Set(0);
  }

  //when button b is pressed, vertical conveyor turns on 
  if(gamepad2.GetRawButtonPressed(2)){
    vConveyorLeft.Set(-1);
    vConveyorRight.Set(1);
  } else {
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);
  }

 
  int dpadDirection = gamepad2.GetPOV(0);
  //if dpad up is pressed, solenoid turns on (out)
  if(dpadDirection == 0){
    rightSolenoid.Set(frc::DoubleSolenoid::kReverse);
    leftSolenoid.Set(frc::DoubleSolenoid::kReverse);
  } //if dpad down is pressed, solenoid turns off (in)
  else if (dpadDirection == 180){
    rightSolenoid.Toggle();
    leftSolenoid.Toggle();
  }
  //scores either one or two cargo, change based on whichever buttons available 
  }

void Robot::joshController() {
 
  //if left trigger is pressed, intake and horizontal conveyor turns on 
  if (gamepad2.GetRawAxis(2) >= 0.05) {
    intake.Set(1);
    hConveyor.Set(1);
  } //if right trigger is pressed, intake and horizontal conveyor will reverse
  else if (gamepad2.GetRawAxis(3) >= 0.05){
    intake.Set(-1);
    hConveyor.Set(-1);
  }
  else {
    intake.Set(0);
    hConveyor.Set(0);
  }
    
  int dpadDirection = gamepad2.GetPOV(0);

  //if dpad up is pressed, vertical conveyor turns on 
  if(dpadDirection == 0){
    vConveyorLeft.Set(1);
    vConveyorRight.Set(1);
  } 
  else {
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);
  }

  // If right bumper is pressed,
  // solenoid sets forward output to false and reverse output to true. 
  if (gamepad2.GetRawButtonPressed(6)){
    
    rightSolenoid.Set(frc::DoubleSolenoid::kReverse);
    leftSolenoid.Set(frc::DoubleSolenoid::kReverse);
  } //Once solenoids are set and left left bumper is pressed, extend pistons (toggle)
  else if (gamepad2.GetRawButtonPressed(5)){
    //Toggle
    rightSolenoid.Toggle();
    leftSolenoid.Toggle();
  }
  //if x button is pressed, robot will socre cargo in low goal
  if (gamepad2.GetRawButtonPressed(3)){
    lowerPortShot();
  } //if y button is pressed, robot will score cargo in upper goal (change buttons in method)
  
  //if a button is pressed, robot will allign with hub
  if (gamepad2.GetRawButtonPressed(1)){
    limelightAlign();
  }
}

void Robot::testController () {
  if(gamepad2.GetRawButton(1)) {
    flywheelPID.SetReference(-DistanceToRPM(LimelightDistance()), rev::ControlType::kVelocity);
  } 
  else {
    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
  }

  //horizontal and vertical reverse
  

  if(gamepad2.GetRawButton(3)) {
    microswitchIntake();
  }

  
  //limelightAlign();
  

  //horizontal conveyor
  if (gamepad2.GetRawAxis(2) >= 0.05) {
    intake.Set(-1);
    hConveyor.Set(-1);
  } 
  else if (gamepad2.GetRawButton(2)) {
    intake.Set(1);
    hConveyor.Set(1);
  } 
  else { 
    intake.Set(0);
    hConveyor.Set(0);
  }

  //vertical conveyor 
  if(gamepad2.GetRawAxis(3) >= 0.05){
    vConveyorLeft.Set(-1);
    vConveyorRight.Set(1);
  }
  else if (gamepad2.GetRawButton(2)) {
    vConveyorLeft.Set(1);
    vConveyorRight.Set(-1);
  } 
  else {
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);
  }

  //intake
 

  //solenoids
  if(gamepad2.GetRawButtonPressed(7)) {
    leftSolenoid.Set(frc::DoubleSolenoid::kForward);
    rightSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
    
  if(gamepad2.GetRawButtonPressed(8)) {
    leftSolenoid.Set(frc::DoubleSolenoid::kReverse);
    rightSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }
}