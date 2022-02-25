// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <units/time.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // test1.Follow(test);
  //encoder1.SetPositionConversionFactor(42);
  testPIDController.SetP(kP);
  testPIDController.SetFF(kFF);
  //testPIDController.SetD(kD);
  //testPIDController.SetI(kI);

  
  
  
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated an  d test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
//   m_autoSelected = m_chooser.GetSelected();
//   // m_autoSelected = SmartDashboard::GetString("Auto Selector",
//   //     kAutoNameDefault);
//   fmt::print("Auto selected: {}\n", m_autoSelected);

//   if (m_autoSelected == kAutoNameCustom) {
//     // Custom Auto goes here
//   } else {
//     // Default Auto goes here
//   }
}

void Robot::AutonomousPeriodic() {
  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  // } else {
  //   // Default Auto goes here
  // }
}

void Robot::TeleopInit() {
  
  exampleDoublePCM.Set(frc::DoubleSolenoid::Value::kForward);
  exampleDoublePCM2.Set(frc::DoubleSolenoid::Value::kForward);


  //testPIDController.SetSmartMotionMaxVelocity(1000);
}

void Robot::TeleopPeriodic() {
    if(gamepad.GetRawButtonPressed(1)) {
      exampleDoublePCM.Toggle();
      exampleDoublePCM2.Toggle();
    }
    else if (gamepad.GetRawButtonPressed(3)) {
      frc::Wait(time1);
      exampleDoublePCM.Set(frc::DoubleSolenoid::Value::kOff);
      exampleDoublePCM2.Set(frc::DoubleSolenoid::Value::kOff);

      
    }
    
    //NOTE: Setting target velocity = setting target RPM
    //testPIDController.SetReference(motorVelocity, rev::ControlType::kVelocity);

          

    /* if(gamepad.GetRawButton(1)) {
      test0.Set(1);
    
    }
    else {
      test0.Set(0);
    } */
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
