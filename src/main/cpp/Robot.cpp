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
#include <frc/Timer.h>
#include <thread>
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

  //flywheel shooter 2 follors flywheel shooter 1
  flywheelShooter2.Follow(flywheelShooter1, true);
  //set PID
  flywheelPID.SetP(kP);
  flywheelPID.SetFF(kFF);
  //left back motor follors left front motor, right back motor follows right front motor
  leftBack.Follow(leftFront);
  rightBack.Follow(rightFront);

     
  

  leftEncoder.SetPositionConversionFactor(42);
  rightEncoder.SetPositionConversionFactor(42);

  //calibrate gyro 
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
    

    functionCompleted = 0;
    encoderDriveRunning = false;
    hasRun = false;
    //resets timer
    pTimer->Reset();

  } else {
    // Default Auto goes here

    //resets encoders and gyro 
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
    gyro.Reset();

    functionCompleted = 0;
    encoderDriveRunning = true;
    
    alignmentComplete = false;

    //resets timers
    aTimer = new frc::Timer();
    sTimer = new frc::Timer();
    
    aTimer->Start();
    sTimer->Start();

    alignElapsedTime = units::second_t(0);  
    elapsedTime = units::second_t(0);
    alignPreviousTime = units::second_t(0);
    previousTime = units::second_t(0);
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
    
    //frc::Wait(units::second_t(3));

    

    //ShootemQuickie()=-'[-345]

    //Output gyro angle and encoder positions to Smart Dashboard 
    frc::SmartDashboard::PutNumber("Gyro Status", gyro.GetAngle());
    frc::SmartDashboard::PutNumber("Right Encoder Status", rightEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Left Encoder Status", leftEncoder.GetPosition());
  } else {
    // Default Auto goes here
    if(encoderDriveRunning) {
      drive(-60, -0.3);
    }

    if ((functionCompleted == 0) && (encoderDriveRunning == false) && (!alignmentComplete)){   
      autoLimelightAlign();
      
    }
    if ((functionCompleted == 0) && (encoderDriveRunning == false) && (alignmentComplete)) {
      shoot();
    }
  }
}

void Robot::TeleopInit() {
  //Set intake to retract upon initilization
  rightSolenoid.Set(frc::DoubleSolenoid::kForward);
  leftSolenoid.Set(frc::DoubleSolenoid::kForward);

  //set motor velocity to 0 
  motorVelocity = 0;
}

//1300 is adequate rpm for low scoring.

void Robot::TeleopPeriodic() {
  tankDrive();
  //joshController();
  //kentController();
  testController();

  frc::SmartDashboard::PutNumber("Set Flywheel RPM * 2", DistanceToRPM(LimelightDistance()));
  frc::SmartDashboard::PutNumber("Distance To Hub", LimelightDistance());
}

void Robot::DisabledInit() {
  rightSolenoid.Set(frc::DoubleSolenoid::kForward);
  leftSolenoid.Set(frc::DoubleSolenoid::kForward);

  leftEncoder.SetPosition(0);
  rightEncoder.SetPosition(0);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  leftSolenoid.Set(frc::DoubleSolenoid::kForward);
  rightSolenoid.Set(frc::DoubleSolenoid::kForward);
  
}

void Robot::TestPeriodic() {
  if(gamepad2.GetRawButtonPressed(1)) {
    limelightAlign();
  }
  else if (gamepad2.GetRawButtonPressed(2)) {
    motorVelocity += 100;  
  }
  else if (gamepad2.GetRawButtonPressed(3)) {
    motorVelocity += 1000;  
  }
  else if (gamepad2.GetRawButtonPressed(4)) {
    motorVelocity = 0;
  }
  else {
    frc::SmartDashboard::PutNumber("Current motor velocity: ", motorVelocity);
  }

  //intake and h conveyor
  if (gamepad2.GetRawButton(5)) {
    hConveyor.Set(-1);
    intake.Set(-1);
  } 
  else if(gamepad2.GetRawButton(6)) {
    hConveyor.Set(1);
    intake.Set(1);
  }
  else {
    hConveyor.Set(0);
    intake.Set(0);
  }


  if (gamepad2.GetRawAxis(2) >= 0.1) {
    flywheelPID.SetReference(DistanceToRPM(LimelightDistance()), rev::ControlType::kVelocity);    
  }
  else {
    flywheelPID.SetReference(0, rev::ControlType::kVelocity);
  }
  if (gamepad2.GetRawAxis(3) >= 0.1) {
    vConveyorLeft.Set(-1);
    vConveyorRight.Set(1);
  }
  else {
    vConveyorLeft.Set(0);
    vConveyorRight.Set(0);
  }
    
  //solenoids
 if(gamepad2.GetRawButtonPressed(7)) {
    leftSolenoid.Set(frc::DoubleSolenoid::kForward);
    rightSolenoid.Set(frc::DoubleSolenoid::kForward);
  }
    
  if(gamepad2.GetRawButtonPressed(8)) {
    leftSolenoid.Set(frc::DoubleSolenoid::kReverse);
    rightSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }

  
  frc::SmartDashboard::PutNumber("Distance to hub: ", LimelightDistance());
  frc::SmartDashboard::PutNumber("Motor Velocity", motorVelocity);
  testController();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif