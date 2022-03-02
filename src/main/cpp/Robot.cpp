// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctime> 
#include <time.h>

//limelight stuff
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <Math.h>

//robot functions
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

/*
* Determines the distance from the limelight's current position to the reflective tape on the goal.
* It also sends the current distance to the SmartDashboard.
*
* Precondition: The reflective tape must be in the limelights POV.
* Postcondition: Returns the distance from the limelight to the reflective tape (x-axis)
*/
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
    
  trueAngle = (34 + targetOffsetAngle_Vertical) * M_PI / 180;

  //distance = (targetHeight - cameraHeight)/tan (mountingAngle + verticalAngleToTarget)
  distanceToHub = (104 - 60)/(tan(trueAngle)) + (24 - 13);

  frc::SmartDashboard::PutNumber("Distance: ", distanceToHub);
  return distanceToHub;
}
void Robot::DistanceToRPM (double distance) {
  distance = distance/12;
  motorVelocity = (int)(4.7 * (distance * distance) - (35.5 * distance) + 1891);
}

void Robot::limelightAlign() {
  //remakes the limelight data table
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto limelight = inst.GetTable("limelight");
  
  //needs to be configured 
  float Kp = 0.1f; 
  float min_command = 0.05f;
  double leftChange, rightChange;
  
  //current speed of the motors
  double leftSpeed = leftFront.GetAppliedOutput();
  double rightSpeed = rightFront.GetAppliedOutput();

  targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);

  //59.6 horizontal degrees FOV
  if(gamepad1.GetRawButton(1)) {
    //3 & -3 are in degrees
    if(targetOffsetAngle_Horizontal > 3) {
      leftChange = -(kP * targetOffsetAngle_Horizontal + 0.05);
      rightChange = kP * targetOffsetAngle_Horizontal + 0.05;

      leftSpeed -= leftChange;      
      rightSpeed += rightChange;
    }
    else if(targetOffsetAngle_Horizontal < -3) {
      leftChange = -(kP * targetOffsetAngle_Horizontal - 0.05);
      rightChange = kP * targetOffsetAngle_Horizontal - 0.05;

      leftSpeed += leftChange;
      rightSpeed -= rightChange;
    }

  }

}

//autonomous functions

//moves the robot the amount specified
void Robot::encoderDrive(double speed, double leftInches, double rightInches, double timeoutSeconds) {
  //amount the robot will move
  int newLeftTarget = (int)(leftInches * countsPerInch);
  int newRightTarget = (int)(rightInches * countsPerInch); 

  //resets the encoders 
  leftEncoder.SetPosition(0);
  rightEncoder.SetPosition(0);

  //ensures the motors are not moving before setting the new position
  rightFront.Set(0);
  rightBack.Set(0);
  leftFront.Set(0);
  leftBack.Set(0);

  //sets the position (amount needed to move) to the new targets (distance)
  rightEncoder.SetPosition(newRightTarget);
  leftEncoder.SetPosition(newLeftTarget);

  //sets both motors to the specified speed
  rightFront.Set(speed);
  leftFront.Set(speed);

  //time::reset;
  //insert timer function (while timer is less than seconds, motor is busy)
  leftFront.Set(speed);
  leftBack.Set(speed);
  rightFront.Set(speed);
  rightBack.Set(speed);

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

//teleop functions (below everything because of trueVelocity)
void Robot::ScoringCargo(){
  if (gamepad2.GetRawButtonPressed(4)){
    //if the Y button to score 1 cargo is pressed...assumes one cargo is already stored in vertical conveyor 
    flywheelShooter1.Set(trueVelocity);
    frc::Wait(units::second_t(1));
    conveyorVerticalLeft.Set(ControlMode::PercentOutput, 1);
    conveyorVerticalRight.Set(ControlMode::PercentOutput, 1);   
    frc::Wait(units::second_t(3));
    //turns on flywheel shooter and vertical conveyor then waits for the robot to score a cargo...
    flywheelShooter1.Set(trueVelocity * 0.75);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.5);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.25);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.1);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(0);

    conveyorVerticalRight.Set(ControlMode::PercentOutput, 0);
    conveyorVerticalLeft.Set(ControlMode::PercentOutput, 0); 
    horizontalConveyor.Set(ControlMode::PercentOutput, 0);
    //flywheel shooter and vertical conveyor are turned off 
  } 
  else if(gamepad2.GetRawButtonPressed(1)){
    //if the A button to score 2 cargo is pressed...assumes both cargo are already stored in conveyor
    flywheelShooter1.Set(trueVelocity);
    frc::Wait(units::second_t(1));
    conveyorVerticalLeft.Set(ControlMode::PercentOutput, 1);
    conveyorVerticalRight.Set(ControlMode::PercentOutput, 1);
    horizontalConveyor.Set(ControlMode::PercentOutput, 1);
    frc::Wait(units::second_t(3));

    //turns on flyhweel shooter, vertical conveyor, and horizontal conveyor then waits for the robot to score both cargo
    flywheelShooter1.Set(trueVelocity * 0.75);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.5);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.25);
    frc::Wait(units::second_t(0.1));
    flywheelShooter1.Set(trueVelocity * 0.1);
    frc::Wait(units::second_t(0.1));

    flywheelShooter1.Set(0);
    conveyorVerticalRight.Set(ControlMode::PercentOutput, 0);
    conveyorVerticalLeft.Set(ControlMode::PercentOutput, 0);
    horizontalConveyor.Set(ControlMode::PercentOutput, 0);
  }
}

void Robot::shoot() {
  DistanceToRPM(LimelightDistance());
  limelightAlign();

  double trueVelocity = motorVelocity * 1.9;

  testPIDController.SetReference(-trueVelocity, rev::ControlType::kVelocity);
 

  frc::Wait(units::second_t(1));

  conveyorVerticalLeft.Set(ControlMode::PercentOutput, 1);
  conveyorVerticalRight.Set(ControlMode::PercentOutput, 1);
  horizontalConveyor.Set(ControlMode::PercentOutput, 1);

  conveyorVerticalLeft.Set(ControlMode::PercentOutput, 0);
  conveyorVerticalRight.Set(ControlMode::PercentOutput, 0);
  horizontalConveyor.Set(ControlMode::PercentOutput, 0);
}

//auto method to intake the ball (get it almost to the flywheel shooter)
void Robot::intake() {
}

void Robot::flywheel(double speed, double rotations, double timeoutSeconds) {
  int newRotationsTarget;
  double flywheelSpeed = speed;
  newRotationsTarget = (int)(rotations * countsPerRev);
  
  flywheelEncoder.SetPosition(0);

  flywheelEncoder.SetPosition(newRotationsTarget);

  //flywheel.Set(pid.Calculate(encoder.GetDistance(), setpoint));
  flywheelShooter1.Set(flywheelSpeed);
  
  //insert timer function (while timer is less than seconds, motor is busy)?

  //timer::reset;

}


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  flywheelShooter2.Follow(flywheelShooter1);
  //encoder1.SetPositionConversionFactor(42);
  testPIDController.SetP(kP);
  testPIDController.SetFF(kFF);
  //testPIDController.SetD(kD);
  //testPIDController.SetI(kI);
}


/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

//Takes the distance (in inches) and converts it to a RPM using the 
//calculated polynomial function 4.7x^2 - 35.5x + 1891 (function comes from the graph made from testing data)


/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here

  }

}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  //Telemetry.addData("Status,", "Resetting Encoders");
  //encoderDrive(1, 120, 120, 2);
  // flywheel(1, 6, 3); //rotations TBD
  // encoderDrive(1, 6, 12, 2); //turn 180 degrees? will need testing 

  //above autonomous code is commented out for tele-op testing purposes


  } else {
    // Default Auto goes here
  }


}

void Robot::TeleopInit() {
  rightSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  leftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
}
void Robot::TeleopPeriodic() {

  if(gamepad1.GetRawButtonPressed(1)) {
    rightSolenoid.Toggle();
    leftSolenoid.Toggle();
  }
  else if(gamepad1.GetRawButtonPressed(2)) {  
    rightSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    leftSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  }

  //flywheel test
  // if(gamepad1.GetRawButtonPressed(1)) {
  //   //DistanceToRPM(Robot::Limelight());
  //   firstRun = Robot::LimelightDistance();
  //   secondRun = Robot::LimelightDistance();
  //   thirdRun = Robot::LimelightDistance();

  //   frc::SmartDashboard::PutNumber("Median", Robot::GetMedian(firstRun, secondRun, thirdRun));
  //   Robot::DistanceToRPM(Robot::GetMedian(firstRun, secondRun, thirdRun));
  // }
  // else if (gamepad1.GetRawButtonPressed(2)) {
  //   //motorVelocity = 2000;
  // }
  // else if(gamepad1.GetRawButtonPressed(3)) {
  //   motorVelocity -= 250;
    
  // }
  // else if (gamepad1.GetRawButtonPressed(4)) {
  //   motorVelocity = 0;
  // }

  // //Calculated gearing Factor of ~1.846 or so
  // trueVelocity = motorVelocity * 1.9;
  // //NOTE: Setting target velocity = setting target RPM
  // testPIDController.SetReference(-trueVelocity, rev::ControlType::kVelocity);
    

  // //limelight
  // // if(gamepad.GetRawButton(1)) {
  // //     Robot::Limelight();
  // // }


 if(gamepad1.GetRawAxis(1) <= 0.1 || gamepad1.GetRawAxis(1) >= 0.1){
   leftFront.Set(gamepad1.GetRawAxis(1));
 }
 else {
   leftFront.Set(0);
 }

 if(gamepad1.GetRawAxis(5) <= 0.1 || gamepad1.GetRawAxis(5) >= 0.1){
   rightFront.Set(-gamepad1.GetRawAxis(5));
 }
 else {
   rightFront.Set(0);
 }

  //Conveyer deliver to flywheel
   if (gamepad2.GetRawButtonPressed(3)) {
     C1Motor.Set(ControlMode::PercentOutput, 1);
     C2Motor.Set(ControlMode::PercentOutput, 1);
   }
   else if (gamepad2_LTrigger) {
     C1Motor.Set(ControlMode::PercentOutput, 1);
     C2Motor.Set(ControlMode::PercentOutput, -1);
   }

   //flywheel activation for purposes of testing

   if(gamepad2.GetRawAxis(3)>= 0.1){
     flywheelShooter1.Set(0.75);

   }else {
     flywheelShooter1.Set(0);
   }
   

   if (gamepad2_AButton){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream", 1);
   } //sets secondary camera stream to the lower right corner of primary camera stream

   if (gamepad1.GetRawButtonPressed(4) == true){
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream", 2);
  } //set primary camera stream to lower right corner of secondary camera stream


//Code for microswitch

  // double cargoLocation = 2;

  //   if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 0){
  //     //if both switches sense cargo...
      
  //     cargoLocation = 1;

  // } else if (horizontalSwitch.Get() == 0 && verticalSwitch.Get() == 1){
  //     //if horizontal switch senses cargo, but vertical switch does not sense cargo...
      
  //     cargoLocation = 2;

  // } else if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 0){
  //     //if horizontal switch does not sense cargo, but vertical switch does sense cargo...
      
  //     cargoLocation = 3;

  // } else if (horizontalSwitch.Get() == 1 && verticalSwitch.Get() == 1){
  //   //if both horizontal and vertical switch do not sense cargo...
    
  //   cargoLocation = 4;

  // } else {
    
  //   cargoLocation = 0;
  // }

  // if (gamepad2.GetRawAxis(2) >= 0.1){
  //   //if left trigger is pressed...
    
  //   if (cargoLocation == 1){
  //     //if both switches sense cargo...
    
  //     conveyorHorizontal.Set(0);
  //     conveyorVertical.Set(0);
  //     //conveyor motors will not turn on because robot already possesses two cargo

  //     ScoringCargo();
  //     //scoring function is called

  //   } else if (cargoLocation == 2){
  //     //if horizontal switch senses cargo and vertical switch does not sense cargo...

  //       while (verticalSwitch.Get() == 1){
  //         conveyorVertical.Set(1);
  //     } //vertical conveyor will turn on until cargo moves to vertical sensor position

  //       while (horizontalSwitch.Get() == 1){
  //         conveyorHorizontal.Set(1);
  //       } //horizontal conveyor will turn on until cargo moves to horizontal sensor position

  //       ScoringCargo();
  //       //scoring function is called
    
  //   } else if (cargoLocation == 3){
  //     //if horizontal switch does not sense cargo and vertical switch senses cargo...

  //       while (horizontalSwitch.Get() == 1){
  //         conveyorHorizontal.Set(1);
  //       } //horizontal conveyor will turn on until cargo moves to horizontal sensor position

  //       ScoringCargo();
  //       //scoring function is called

  //   } else if (cargoLocation == 4){
  //     //if both horizontal and vertical switch do not sense cargo 

  //       while (horizontalSwitch.Get() == 1){
  //         conveyorHorizontal.Set(1);
  //       } //horizontal conveyor will turn on until cargo moves to horizontal sensor position

  //       while (verticalSwitch.Get() == 1){
  //         conveyorVertical.Set(1);

  //       } //vertical conveyor will turn on until cargo moves to vertical sensor position

  //       ScoringCargo();
  //       //scoring function is called

  //   } else {

  //       conveyorHorizontal.Set(0);
  //       conveyorVertical.Set(0);

  //   }

  // }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif