// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "Robot.h"


// #include <fmt/core.h>

// #include "frc/smartdashboard/SmartDashboard.h"
// #include "networktables/NetworkTable.h"
// #include "networktables/NetworkTableInstance.h"
// #include "networktables/NetworkTableEntry.h"
// #include "networktables/NetworkTableValue.h"
// #include "wpi/span.h"
// #include <frc/DoubleSolenoid.h>
// #include <Math.h>   
// #include "cameraserver/CameraServer.h"

// void Robot::RobotInit() {
//   m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//   m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//   frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    
//   // test1.Follow(test);
//   //encoder1.SetPositionConversionFactor(42);
//   testPIDController.SetP(kP);
//   testPIDController.SetFF(kFF);
//   //testPIDController.SetD(kD);
//   //testPIDController.SetI(kI);
  

// }

// /**
//  * This function is called every robot packet, no matter the mode. Use
//  * this for items like diagnostics that you want ran during disabled,
//  * autonomous, teleoperated an  d test.
//  *
//  * <p> This runs after the mode specific periodic functions, but before
//  * LiveWindow and SmartDashboard integrated updating.
//  */

// // void Robot::SetFollowers() {
// //   L2Motor.Follow(L1Motor);
// //   R2Motor.Follow(R1Motor);
// // }


// void Robot::RobotPeriodic() {
//     auto inst = nt::NetworkTableInstance::GetDefault();
//     auto table = inst.GetTable("limelight");

//     table->PutNumber("stream", 1);
// }

// /**
//  * This autonomous (along with the chooser code above) shows how to select
//  * between different autonomous modes using the dashboard. The sendable chooser
//  * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
//  * remove all of the chooser code and uncomment the GetString line to get the
//  * auto name from the text box below the Gyro.
//  *
//  * You can add additional auto modes by adding additional comparisons to the
//  * if-else structure below with additional strings. If using the SendableChooser
//  * make sure to add them to the chooser code above as well.
//  */
// void Robot::AutonomousInit() {
// //   m_autoSelected = m_chooser.GetSelected();
// //   // m_autoSelected = SmartDashboard::GetString("Auto Selector",
// //   //     kAutoNameDefault);
// //   fmt::print("Auto selected: {}\n", m_autoSelected);

// //   if (m_autoSelected == kAutoNameCustom) {
// //     // Custom Auto goes here
// //   } else {
// //     // Default Auto goes here
// //   }
// }

// void Robot::AutonomousPeriodic() {
//   // if (m_autoSelected == kAutoNameCustom) {
//   //   // Custom Auto goes here
//   // } else {
//   //   // Default Auto goes here
//   // }
// }

// void Robot::TeleopInit() {
  

//   //.Set(frc::DoubleSolenoid::Value::kForward);

//   //testPIDController.SetSmartMotionMaxVelocity(1000);


// }

// void Robot::TeleopPeriodic() {
//     //Drive + slow mode
// //     if (LStick >= 0.1 || LStick <= -0.1) {
// //       L1Motor.Set(-LStick);
// //     }
// //     if (RStick >= 0.1 || RStick <= -0.1){
// //       R1Motor.Set(-RStick);333
// //     }
// //     if (RTrigger >= 0.1){
// //       R1Motor.Set(R1Motor.GetAppliedOutput()/2);
// //       L1Motor.Set(L1Motor.GetAppliedOutput()/2);
// //     }
// //  // Conveyer deliver to flywheel
// //       if (gamepad2.GetRawButtonPressed(3)) {
// //         C1Motor.Set(ControlMode::PercentOutput, 1);
// //         C2Motor.Set(ControlMode::PercentOutput, 1);
// //       }
// //       else if (LTrigger == true) {
// //         C1Motor.Set(ControlMode::PercentOutput, 1);
// //         C2Motor.Set(ControlMode::PercentOutput, -1);
// //       }

//     //frc::CameraServer::StartAutomaticCapture();

    
//      //sets secondary camera stream to the lower right corner of primary camera stream
//      //set primary camera stream to lower right corner of secondary camera stream
  
  

//     /* if(gamepad.GetRawButton(1)) {
//       test0.Set(1);
    
//     }
//     else {
//       test0.Set(0);
//     } */
// }

// void Robot::DisabledInit() {}

// void Robot::DisabledPeriodic() {}

// void Robot::TestInit() {}

// void Robot::TestPeriodic() {}

// // double Robot::Limelight() {
// //     auto inst = nt::NetworkTableInstance::GetDefault();
// //     auto limelight = inst.GetTable("limelight");

// //     targetOffsetAngle_Horizontal = limelight->GetNumber("tx",0.0);
// //     targetOffsetAngle_Vertical = limelight->GetNumber("ty",0.0);
// //     targetArea = limelight->GetNumber("ta",0.0);
// //     targetSkew = limelight->GetNumber("ts",0.0);

// //     frc::SmartDashboard::PutNumber("Horizontal:", targetOffsetAngle_Horizontal);
// //     frc::SmartDashboard::PutNumber("Vertical:", targetOffsetAngle_Vertical);
// //     frc::SmartDashboard::PutNumber("area:", targetArea);
// //     frc::SmartDashboard::PutNumber("skew:", targetSkew);
    
// //     trueAngle = (34 + targetOffsetAngle_Vertical) * M_PI / 180;

// //     //should be 38 off the ground, but height is different for testing purposes
// //     distanceToHub = (104-60)/(tan(trueAngle)) + (24 - 13);

// //     frc::SmartDashboard::PutNumber("Distance: ", distanceToHub);
// //     return distanceToHub;
// //     //distance = (targetHeight - cameraHeight)/tan (mountingAngle + verticalAngleToTarget)
// // }

// // void Robot::DistanceToRPM (double distance) {
  
// //   distance = distance/12;
// //   motorVelocity = (int)(4.7*(distance*distance) - (35.5*distance) + 1891);
// // }

// // double Robot::GetMedian(double value1, double value2, double value3) {
// //   if(value1 > value2 && value1 > value3) {
// //     if(value2 > value3) {
// //       return value2;
// //     }
// //     else {
// //       return value3;
// //     }
// //   }
// //   else if(value2 > value1 && value2 > value3) {
// //     if(value1 > value3) {
// //       return value1;
// //     }
// //     else {
// //       return value3;
// //     }
// //   }
// //   else if(value3 > value1 && value3 > value2) {
// //     if(value1 > value2) {
// //       return value1;
// //     }
// //     else {
// //       return value2;
// //     }
// //   }
// //   else if(value1 == value3 || value1 == value2) {
// //     return value1;
// //   }
// //   else {
// //     return value2;
// //   }
// // }



// #ifndef RUNNING_FRC_TESTS
// int main() {
//   return frc::StartRobot<Robot>();
// }
// #endif
