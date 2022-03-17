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

frc::Timer *sTimer;
frc::Timer *aTimer;

double Robot::LimelightDistance() {
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto limelight = inst.GetTable("limelight");

    targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
    targetOffsetAngle_Vertical = limelight->GetNumber("ty",0.0);
    targetArea = limelight->GetNumber("ta",0.0);
    targetSkew = limelight->GetNumber("ts",0.0);

    frc::SmartDashboard::PutNumber("Horizontal:", targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("Vertical:", targetOffsetAngle_Vertical);
    frc::SmartDashboard::PutNumber("area:", targetArea);
    frc::SmartDashboard::PutNumber("skew:", targetSkew);
    
    trueAngle = (39 + targetOffsetAngle_Vertical) * M_PI / 180;

    //should be 38 off the ground, but height is different for testing purposes
    distanceToHub = (104-34)/(tan(trueAngle)) + (24 + 13);

    frc::SmartDashboard::PutNumber("Distance: ", distanceToHub);
    return distanceToHub;
    //distance = (targetHeight - cameraHeight)/tan (mountingAngle + verticalAngleToTarget)
}

double Robot::GetMedian(double value1, double value2, double value3) {
 
  if(value1 > value2 && value1 > value3) {
    if(value2 > value3) {
      return value2;
    }
    else {
      return value3;
    }
  }
  else if(value2 > value1 && value2 > value3) {
    if(value1 > value3) {
      return value1;
    }
    else {
      return value3;
    }
  }
  else if(value3 > value1 && value3 > value2) {
    if(value1 > value2) {
      return value1;
    }
    else {
      value2;
    }
  }
  else if(value1 == value3 || value1 == value2) {
    return value1;
  }
  else {
    return value2;
  }
  
}

//the distance can be a positive or a negative
//input a negative speed to go backwards, positive to go forwards
void Robot::drive(double distance, double speed) {
  gyroAngle = gyro.GetAngle();
  targetDistance = distance * countsPerInch;
  averageActualDistance = (-(leftEncoder.GetPosition()) + rightEncoder.GetPosition())/2;
  speedChange = averageActualDistance/targetDistance;
  fractionOfTargetDistance = targetDistance * (1.0/3); //to be adjusted
  
  speedFactor = error * 0.1; //arbitrary number (to be tested)

  if (abs(leftEncoder.GetPosition()) < abs(targetDistance) && abs(rightEncoder.GetPosition()) < abs(targetDistance)){
    if(error > 1){
      rightFront.Set(speed + (speed * speedFactor));
      leftFront.Set(-speed);
    } else if (error < -1) {
      leftFront.Set(-(speed + (speed * speedFactor)));
      rightFront.Set(speed);      
    } else {
      rightFront.Set(speed);
      leftFront.Set(-speed);   
    }

    if (abs(leftEncoder.GetPosition()) >= abs(fractionOfTargetDistance) && abs(rightEncoder.GetPosition()) >= abs(fractionOfTargetDistance)) {
      speed /= 8;     
    }
  } else if (abs(leftEncoder.GetPosition()) < abs(targetDistance) && abs(rightEncoder.GetPosition()) < abs(targetDistance)) {
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
      speed /= 2;     
    }
  } else {
    leftFront.Set(0);
    rightFront.Set(0);

    encoderDriveRunning = false;
    sTimer->Reset();
    aTimer->Reset();

    frc::SmartDashboard::PutNumber("Left Encoder value", -leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Right Encoder value", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("target", targetDistance);
  }
}  

void Robot::limelightAlign() {
  //remakes the limelight data table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto limelight = inst.GetTable("limelight");
  
  //needs to be configured 
  float constant = 0.01; 
  float min_command = 0.05f;
  double leftChange, rightChange;

  //current speed of the motors
  double leftSpeed = leftFront.GetAppliedOutput();
  double rightSpeed = rightFront.GetAppliedOutput();

  targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
  frc::SmartDashboard::PutNumber("horizontal offset", targetOffsetAngle_Horizontal);

  //59.6 horizontal degrees FOV
  if(gamepad1.GetRawButton(3)) {
    //3 & -3 are in degrees
    if(targetOffsetAngle_Horizontal > 3) {
      leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
      rightChange = -(constant * abs(targetOffsetAngle_Horizontal) + 0.1);

      leftSpeed = leftChange;      
      rightSpeed = rightChange;

      frc::SmartDashboard::PutNumber("left motor speed", leftSpeed);
      frc::SmartDashboard::PutNumber("left motor speed", rightSpeed);

      leftFront.Set(-leftSpeed);
      rightFront.Set(rightSpeed);
    }
    else if(targetOffsetAngle_Horizontal < -3) {
      leftChange = (constant * abs(targetOffsetAngle_Horizontal) + 0.1);
      rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

      leftSpeed = leftChange;
      rightSpeed = rightChange;

      frc::SmartDashboard::PutNumber("left motor speed", leftSpeed);
      frc::SmartDashboard::PutNumber("left motor speed", rightSpeed);
      
      leftFront.Set(leftSpeed);
      rightFront.Set(rightSpeed);
    }
    
  }
  else {
    leftFront.Set(0);
    rightFront.Set(0);
  }

  if(gamepad1.GetRawButton(3)) {
    frc::SmartDashboard::PutNumber("left motor speed", leftSpeed);
  }
  

}

void Robot::autoLimelightAlign() {
  //remakes the limelight data table
  //timer testing
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

      targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
      frc::SmartDashboard::PutNumber("horizontal offset", targetOffsetAngle_Horizontal);
      
        if((targetOffsetAngle_Horizontal > 3)) {
          leftChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;
          rightChange = -(constant * abs(targetOffsetAngle_Horizontal) + 0.1);

          leftSpeed = leftChange;      
          rightSpeed = rightChange;

          frc::SmartDashboard::PutNumber("left motor speed", leftSpeed);
          frc::SmartDashboard::PutNumber("left motor speed", rightSpeed);

          leftFront.Set(-leftSpeed);
          rightFront.Set(rightSpeed);
        }
        else if((targetOffsetAngle_Horizontal < -3)) {
          leftChange = (constant * abs(targetOffsetAngle_Horizontal) + 0.1);
          rightChange = constant * abs(targetOffsetAngle_Horizontal) + 0.1;

          leftSpeed = leftChange;
          rightSpeed = rightChange;

          frc::SmartDashboard::PutNumber("left motor speed", leftSpeed);
          frc::SmartDashboard::PutNumber("left motor speed", rightSpeed);
          
          leftFront.Set(leftSpeed);
          rightFront.Set(rightSpeed);
        } else {
          leftFront.Set(0);
          rightFront.Set(0);
          alignmentComplete = true;
          //delete aTimer;
        }

        
      alignPreviousTime = alignTime; 

    
  }
}

void Robot::shoot(){
  
  //autoLimelightAlign();
 
  
  DistanceToRPM(LimelightDistance());
  flywheelPID.SetReference(-motorVelocity, rev::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("MotorVelocity", motorVelocity);
  
  
  auto time = sTimer->Get();  
  
  elapsedTime += (time - previousTime);

  if (elapsedTime >= units::second_t(5)) {
    intake.Set(ControlMode::PercentOutput, 0);
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
    vConveyorRight.Set(ControlMode::PercentOutput, 0);
    hConveyor.Set(ControlMode::PercentOutput, 0);

    flywheelPID.SetReference(0, rev::ControlType::kVelocity);

    functionCompleted = 1;
    //delete sTimer;
  }
  else if (elapsedTime >= units::second_t(3)) {
    intake.Set(ControlMode::PercentOutput, 1);
    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
    vConveyorRight.Set(ControlMode::PercentOutput, -1);
    hConveyor.Set(ControlMode::PercentOutput, 1);
  }
  
  previousTime = time; 
}
  

void Robot::encoderDrive(double speed, double leftInches, double rightInches, double timeoutSeconds) {
  int newLeftTarget;
  int newRightTarget;
  double rightPower = speed;
  double leftPower = speed;

  newLeftTarget = (int)(leftInches * countsPerInch);
  newRightTarget = (int)(rightInches * countsPerInch); 

  leftEncoder.SetPosition(0);
  rightEncoder.SetPosition(0);

  rightFront.Set(0);
  rightBack.Set(0);
  leftFront.Set(0);
  leftBack.Set(0);

  rightEncoder.SetPosition(newRightTarget);
  leftEncoder.SetPosition(newLeftTarget);

  rightFront.Set(rightPower);
  leftFront.Set(leftPower);

  //time::reset;

  //insert timer function (while timer is less than seconds, motor is busy)?

  leftFront.Set(leftPower);
  leftBack.Set(leftPower);
  rightFront.Set(rightPower);
  rightBack.Set(rightPower);

  leftEncoder.GetPosition();
  rightEncoder.GetPosition();
  }

void Robot::turnDrive(double speed, double degrees, double timeoutSeconds) {
  int newLeftTarget;
  int newRightTarget;
  double rightPower = speed;
  double leftPower = speed;

  if (degrees > 0) {
    newLeftTarget = (int)(degreesToInches * countsPerInch);
    newRightTarget = -(int)(degreesToInches * countsPerInch);
    rightPower = -speed;
    leftPower = speed;

  } else{
    newLeftTarget = -(int)(degreesToInches * countsPerInch);
    newRightTarget = (int)(degreesToInches * countsPerInch);
    rightPower = speed;
    leftPower = -speed;
  }
}

void Robot::ScoringCargo(){
  if (gamepad2.GetRawButtonPressed(4)){
    //if the Y button to score 1 cargo is pressed...assumes one cargo is already stored in vertical conveyor 
    flywheelShooter1.Set(1);
    //possibly add a wait command as flywheel rpm increases to shooting speed?
    //vConveyorLeft.Set(1);
    frc::Wait(units::second_t(3));
    //turns on flywheel shooter and vertical conveyor then waits for the robot to score a cargo...
    flywheelShooter1.Set(0);
    //vConveyorLeft.Set(0);
    //flywheel shooter and vertical conveyor are turned off 
  }
  if (gamepad2.GetRawButtonPressed(1)){
    //if the A button to score 2 cargo is pressed...assumes both cargo are already stored in conveyor
    flywheelShooter1.Set(1);
    //possibly add a wait command as flywheel rpm increases to shooting speed
    //vConveyorLeft.Set(1);
    ////conveyorHorizontal.Set(1);
    frc::Wait(units::second_t(3));
    //turns on flyhweel shooter, vertical conveyor, and horizontal conveyor then waits for the robot to score both cargo
    flywheelShooter1.Set(0);
    //vConveyorLeft.Set(0);
    //flywheel shooter, vertical conveyor, and horizontal conveyor are all turned off
  }
}

void Robot::ShootemQuickie() {
  // bool x;
  //   if(gamepad1.GetRawButtonPressed(4)) {
  //   x = 1;
  // }
    
    
    //Calculated gearing factor of ~1.846 or so
    
    //NOTE: Setting target velocity = setting target RPM
    isActive = true;
    hConveyor.Set(ControlMode::PercentOutput, 1);
    //DistanceToRPM(LimelightDistance());
    trueVelocity = motorVelocity * 2.123;
    flywheelPID.SetReference(-trueVelocity, rev::ControlType::kVelocity);
    frc::Wait(units::second_t(1));

    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
    vConveyorRight.Set(ControlMode::PercentOutput,-1);
    
    frc::Wait(units::second_t(4));
    
    intake.Set(ControlMode::PercentOutput, 0);
    hConveyor.Set(ControlMode::PercentOutput, 0);
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
    vConveyorRight.Set(ControlMode::PercentOutput, 0);

    // flywheelPID.SetReference(-trueVelocity/2, rev::ControlType::kVelocity);
    // frc::Wait(units::second_t(1));
    // flywheelPID.SetReference(-trueVelocity/4, rev::ControlType::kVelocity);
    // frc::Wait(units::second_t(1));
    // flywheelPID.SetReference(-trueVelocity/8, rev::ControlType::kVelocity);
    // frc::Wait(units::second_t(1));
    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
    
    //x = 0;
    motorVelocity = 0;
    isActive = false;
    
  frc::SmartDashboard::PutNumber("Set rpm", motorVelocity);
  frc::SmartDashboard::PutNumber("True rpm", trueVelocity);

}

void Robot::lowerPortShot() {    
    //NOTE: Setting target velocity = setting target RPM
    isActive = true;

    hConveyor.Set(ControlMode::PercentOutput, 1);
    //DistanceToRPM(LimelightDistance());
    flywheelPID.SetReference(-1300, rev::ControlType::kVelocity);
    frc::Wait(units::second_t(1));

    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
    vConveyorRight.Set(ControlMode::PercentOutput,-1);
    
    frc::Wait(units::second_t(4));
    
    intake.Set(ControlMode::PercentOutput, 0);
    hConveyor.Set(ControlMode::PercentOutput, 0);
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
    vConveyorRight.Set(ControlMode::PercentOutput, 0);

    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
    
    isActive = false;
    
  frc::SmartDashboard::PutNumber("Set rpm", motorVelocity);
  frc::SmartDashboard::PutNumber("True rpm", trueVelocity);
}

// takes distance (in inches) and converts it to a rpm using
// calculated polynomial function 4.7x^2 - 35.5x + 1891
void Robot::DistanceToRPM (double distance) {
  distance = distance/12;
  motorVelocity = (int)(5.76*(distance*distance) + (5.39*distance) + 3670);
}

int Robot::getPosition() {
    if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 0){
        //if both switches sense cargo...
        return 1;

    } else if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 1){
    //if horizontal switch senses cargo, but vertical switch does not sense cargo...
        return 2;

    } else if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 0){
    //if horizontal switch does not sense cargo, but vertical switch does sense cargo...
        return 3;

    } else if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 1){
    //if both horizontal and vertical switch do not sense cargo...
        return 4;

    } else {
        return  0;
    }

}

void::Robot::troyAndMichaelController(){
   //left motors
    if(Attack31.GetRawButtonPressed(1) && (Attack31.GetRawAxis(1) >= 0.1 || Attack31.GetRawAxis(1) <= -0.1)) {
        leftFront.Set(Attack31.GetRawAxis(1) * 0.5);
    }
    else if(Attack31.GetRawAxis(1) >= 0.1 || Attack31.GetRawAxis(1) <= -0.1) {
        leftFront.Set(Attack31.GetRawAxis(1));
    }
    else {
        leftFront.Set(0);
    }   
  
    //Right motors
    if(Attack32.GetRawButtonPressed(1) && (Attack32.GetRawAxis(1) >= 0.1 || Attack32.GetRawAxis(1) <= -0.1)) {
        rightFront.Set(-Attack32.GetRawAxis(1) * 0.5);
    }
    else if(Attack32.GetRawAxis(1) >= 0.1 || Attack32.GetRawAxis(1) <= -0.1) {
        rightFront.Set(-Attack32.GetRawAxis(1));
    }
    else {
        rightFront.Set(0);
    }   
}

//Kent Prefrence

void::Robot::kentController(){

  //when right bumper is pressed, turn on intake and horizontal conveyor
  if(gamepad2.GetRawButtonPressed(6)){
    intake.Set(ControlMode::PercentOutput, 1);
    hConveyor.Set(ControlMode::PercentOutput, 1);
  } //when left bumper is pressed, reverse intake and horizontal conveyor
  else if(gamepad2.GetRawButtonPressed(5)){ 
    intake.Set(ControlMode::PercentOutput, -1);
    hConveyor.Set(ControlMode::PercentOutput, -1);
  } //turn off intake and horizontal conveyor otherwise
  else {
    intake.Set(ControlMode::PercentOutput, 0);
    hConveyor.Set(ControlMode::PercentOutput, 0);
  }

  //when button b is pressed, vertical conveyor turns on 
  if(gamepad2.GetRawButtonPressed(2)){
    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
    vConveyorRight.Set(ControlMode::PercentOutput, 1);
  } else {
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
    vConveyorRight.Set(ControlMode::PercentOutput, 0);
  }

 
  int dpadDirection = gamepad2.GetPOV(0);
  //if dpad up is pressed, solenoid turns on (out)
  if(dpadDirection == 0){
    //rightSolenoid.Toggle();
    //leftSolenoid.Toggle();
  } //if dpad down is pressed, solenoid turns off (in)
  else if (dpadDirection ==180){
    //rightSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    //leftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  }
  //scores either one or two cargo, change based on whichever buttons available 
  ScoringCargo();
  }

void Robot::joshController() {
 
  //if left trigger is pressed, intake and horizontal conveyor turns on 
    if (gamepad2.GetRawAxis(2) >=0;05) {
      intake.Set(ControlMode::PercentOutput, 1);
      hConveyor.Set(ControlMode::PercentOutput, 1);
    } //if right trigger is pressed, intake and horizontal conveyor will reverse
    else if (gamepad2.GetRawAxis(3) >=0;05){
      intake.Set(ControlMode::PercentOutput, -1);
      hConveyor.Set(ControlMode::PercentOutput, -1);
    }
    else {
      intake.Set(ControlMode::PercentOutput, 0);
      hConveyor.Set(ControlMode::PercentOutput, 0);
    }
    
    int dpadDirection = gamepad2.GetPOV(0);

  //if dpad up is pressed, vertical conveyor turns on 
  if(dpadDirection == 0){
    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
    vConveyorRight.Set(ControlMode::PercentOutput, 1);
  } else {
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
    vConveyorRight.Set(ControlMode::PercentOutput, 0);
  }

  //if right bumper is pressed, solenoid turns on (out)
  if (gamepad2.GetRawButtonPressed(6)){
    //rightSolenoid.Toggle();
    //leftSolenoid.Toggle();
  } //if left bumper is pressed, solenoid turns off (in)
  else if (gamepad2.GetRawButtonPressed(5)){
    //rightSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    //leftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  }
  //if x button is pressed, robot will socre cargo in low goal
  if (gamepad2.GetRawButtonPressed(3)){
    lowerPortShot();
  } //if y button is pressed, robot will score cargo in upper goal (change buttons in method)
  if (gamepad2.GetRawButtonPressed(4)){
    ScoringCargo();
  } //if a button is pressed, robot will allign with hub
  if (gamepad2.GetRawButtonPressed(1)){
    limelightAlign();
  }
}