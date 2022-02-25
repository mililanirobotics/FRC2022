// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include <frc/DoubleSolenoid.h>
#include "ctre/phoenix.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  double Limelight();
  //motors
  rev::CANSparkMax test0{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax test1{16, rev::CANSparkMax::MotorType::kBrushless};
  
  TalonSRX test{10};
  
  //pneumatics
  frc::DoubleSolenoid exampleDoublePCM{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  frc::DoubleSolenoid exampleDoublePCM2{frc::PneumaticsModuleType::CTREPCM, 2, 3};

  //encoders
  rev::SparkMaxRelativeEncoder encoder1 = test0.GetEncoder();

  //pidcontroller  
  rev::SparkMaxPIDController testPIDController = test0.GetPIDController();

  //controller
  frc::Joystick gamepad{0};
  
  //kP = 0.000001, kFF 0.0001695
  //Change the feed forward to adjust settling below or above the set point
  //Tuning a PID for future reference:  
  //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-pid-controller.html
  double kP = 0.000001, kFF = 0.00017375, kMaxoutput = 1, kminoutput = -1, kI = 1e-4, kD = 1;
  double motorVelocity = 0;
  units::second_t time1{0.2};
  
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
