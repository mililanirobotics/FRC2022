// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctime> 

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

    //should be 38 off the ground, but height is different for testing purposes
    distanceToHub = (104-60)/(tan(trueAngle)) + (24 - 13);

    frc::SmartDashboard::PutNumber("Distance: ", distanceToHub);
    return distanceToHub;
    //distance = (targetHeight - cameraHeight)/tan (mountingAngle + verticalAngleToTarget)
}

// takes distance (in inches) and converts it to a rpm using
// calculated polynomial function 4.7x^2 - 35.5x + 1891
void Robot::DistanceToRPM (double distance) {
  
  distance = distance/12;
  motorVelocity = (int)(4.7*(distance*distance) - (35.5*distance) + 1891);
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
  } else {
    // Default Auto goes here

  }

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


void Robot::horizontalConveyor(double speed, double rotations, double timeoutSeconds) {
  int newRotationsTarget;
  double conveyorSpeed = speed;
  newRotationsTarget = (int)(rotations * countsPerRev);
  
  horizontalConveyorEncoder.SetPosition(0);

  horizontalConveyorEncoder.SetPosition(newRotationsTarget);

  conveyorHorizontal.Set(conveyorSpeed);
  
  //timer::reset;
  
  //insert timer function (while timer is less than seconds, motor is busy)?

  conveyorHorizontal.Set(0);

  horizontalConveyorEncoder.GetPosition();

}

void Robot::verticalConveyor(double speed, double rotations, double timeoutSeconds) {
  int newRotationsTarget;
  double conveyorSpeed = speed;
  newRotationsTarget = (int)(rotations * countsPerRev);
  
  verticalConveyorEncoder.SetPosition(0);

  verticalConveyorEncoder.SetPosition(newRotationsTarget);

  conveyorVertical.Set(conveyorSpeed);
  
  //timer::reset;
  
  //insert timer function (while timer is less than seconds, motor is busy)?

  conveyorVertical.Set(0);

  verticalConveyorEncoder.GetPosition();
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




void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  //Telemetry.addData("Status,", "Resetting Encoders");
  encoderDrive(1, 120, 120, 2);
  turnDrive(1, 180, 2);
  horizontalConveyor(1, 5, 3);   //rotations TBD
  verticalConveyor(1, 5, 3);   //rotations TBD
  flywheel(1, 6, 3); //rotations TBD
  encoderDrive(1, 6, 12, 2); //turn 180 degrees? will need testing 


  } else {
    // Default Auto goes here
  }


}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  //joysticks
  // double leftStickY = gamepad.GetRawAxis(1);
  // double rightStickY = gamepad.GetRawAxis(5);

  //buttons
  // bool b = gamepad.GetRawButton(2);
  // bool y = gamepad.GetRawButton(4);
  // bool x = gamepad.GetRawButton(3);

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
  

  //flywheel test
  if(gamepad1.GetRawButtonPressed(1)) {
    //DistanceToRPM(Robot::Limelight());
    firstRun = Robot::LimelightDistance();
    secondRun = Robot::LimelightDistance();
    thirdRun = Robot::LimelightDistance();

    frc::SmartDashboard::PutNumber("Median", Robot::GetMedian(firstRun, secondRun, thirdRun));
    Robot::DistanceToRPM(Robot::GetMedian(firstRun, secondRun, thirdRun));
  }
  else if (gamepad1.GetRawButtonPressed(2)) {
    //motorVelocity = 2000;
  }
  else if(gamepad1.GetRawButtonPressed(3)) {
    motorVelocity -= 250;
    
  }
  else if (gamepad1.GetRawButtonPressed(4)) {
    motorVelocity = 0;
  }

  //Calculated gearing Factor of ~1.846 or so
  trueVelocity = motorVelocity * 1.9;
  //NOTE: Setting target velocity = setting target RPM
  testPIDController.SetReference(-trueVelocity, rev::ControlType::kVelocity);
    

  //limelight
  // if(gamepad.GetRawButton(1)) {
  //     Robot::Limelight();
  // }

  //actual teleop
    //Drive + slow mode

  //Left motors
  if(gamepad2_RTrigger >= 0.1 && (gamepad1_LStick >= 0.1 || gamepad1_LStick <= -0.1)) {
    leftFront.Set(-gamepad1_LStick * 0.5);
  }
  else if(gamepad1_LStick >= 0.1 || gamepad1_LStick <= -0.1) {
    leftFront.Set(-gamepad1_LStick);
  }
  else {
    leftFront.Set(0);
  }
  
  //Right motors
  if(gamepad2_RTrigger >= 0.1 && (gamepad1_RStick >= 0.1 || gamepad1_LStick <= -0.1)) {
    rightFront.Set(-gamepad1_RStick * 0.5);
  }
  else if(gamepad1_RStick >= 0.1 || gamepad1_RStick <= -0.1) {
    rightFront.Set(-gamepad1_RStick);
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

  //frc::CameraServer::StartAutomaticCapture();

  if (gamepad2_AButton){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream", 1);
  } //sets secondary camera stream to the lower right corner of primary camera stream

  if (gamepad1.GetRawButtonPressed(4) == true){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("stream", 2);
  } //set primary camera stream to lower right corner of secondary camera stream

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