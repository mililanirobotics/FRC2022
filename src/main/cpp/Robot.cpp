// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include "Robot.h"
#include "Functions.h"

#include <fmt/core.h>
#include <units/time.h>

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

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  flywheelShooter2.Follow(flywheelShooter1);
  flywheelPID.SetP(kP);
  flywheelPID.SetFF(kFF);

  leftBack.Follow(leftFront);
  rightBack.Follow(rightFront);

  leftEncoder.SetPositionConversionFactor(42);
  rightEncoder.SetPositionConversionFactor(42);

  

  gyro.Calibrate();
}


/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  
}





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
    // 
    distance = frc::SmartDashboard::GetNumber("Distance In Inches", 12);
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
    gyro.Reset();
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    calculateRotateValue(72, 0.2); //add extra 12 inches from wanted distance 
    frc::SmartDashboard::PutNumber("Gyro Status", gyro.GetAngle());
    frc::SmartDashboard::PutNumber("Right Encoder Status", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Left Encoder Status", leftEncoder.GetPosition());
  } else {
    // Default Auto goes here
    
  }


}

void Robot::TeleopInit() {
  flywheelShooter2.Follow(flywheelShooter1);
  motorVelocity = 0;
}

//1300 is adequate rpm for low scoring.

void Robot::TeleopPeriodic() {
  // if(gamepad1.GetRawButtonPressed(2)) {
  //   LimelightDistance();
  // }
  // if(gamepad2.GetRawButton(3)) {
  //   intake.Set(ControlMode::PercentOutput, 1);
  // }
  // else {
  //   intake.Set(ControlMode::PercentOutput, 0);
  // }
 
  // if(gamepad2.GetRawButtonPressed(3)) {
  //   motorVelocity += 1000;
  //   frc::SmartDashboard::PutNumber("set rpm", motorVelocity);
  // }
  // if(gamepad2.GetRawButtonPressed(10)){
  //   motorVelocity += 100;
  //   frc::SmartDashboard::PutNumber("set rpm", motorVelocity);
  // }
  //limelight align
  //ShootemQuickie();
  
  //Left motors
  frc::SmartDashboard::PutNumber("Distance: ", LimelightDistance());

  tankDrive();
 
  //intake
  intakeEm();

  //align
  if(gamepad2.GetRawButton(4)) {
    limelightAlign();
  }

  //shoot
  if(gamepad2.GetRawButton(2)) {
    //ShootemQuickie();
    vConveyorRight.Set(ControlMode::PercentOutput, -1);
    vConveyorLeft.Set(ControlMode::PercentOutput, 1);
  }
  else {
    vConveyorRight.Set(ControlMode::PercentOutput, 0);
    vConveyorLeft.Set(ControlMode::PercentOutput, 0);
  }
 
  
  if(gamepad2.GetRawButtonPressed(1)) {
    DistanceToRPM(LimelightDistance());
    frc::SmartDashboard::PutNumber("motorVelocity: ", motorVelocity);
    flywheelPID.SetReference(-motorVelocity, rev::ControlType::kVelocity);
  }
  
  if(gamepad2.GetRawButtonPressed(9)) {
    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
  }

  //lower port 
  // if(gamepad2.GetRawButton(3)) {
  //   lowerPortShot();
  // }
 
  // if(b && leftStickY >= 0.05) {
  //   L1Motor.Set(-leftStickY * 0.5);

  //   L2Motor.Set(-leftStickY * 0.5);
  // }
  // else if(y && leftStickY >= 0.05) {
  //   L1Motor.Set(-leftStickY);
  //   L2Motor.Set(-leftStickY);
  // }

  // if(rightStickY >= 0.05) {
  //   L1Motor.Set(-rightStickY);
  //   L2Motor.Set(-rightStickY);
  // }
  
  //actual teleop
    //Drive + slow mode

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