// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
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

  //user defined functions
  double LimelightDistance();
  double GetMedian(double value1, double value2, double value3);
  void DistanceToRPM(double distance);
  void limelightAlign();
  void ScoringCargo();

  //autonomous functions
  void encoderDrive(double speed, double leftInches, double rightInches, double timeoutSeconds);
  //void flywheel(double speed, double rotations, double timeoutSeconds);
  void turnDrive(double speed, double degrees, double timeoutSeconds);
  void shoot(); //needs to be finished
  void intake(); //needs to be finished
  void MicroswitchIntake();
  //CIMs  
  TalonSRX conveyorVerticalLeft{20};
  TalonSRX conveyorVerticalRight{19};
  TalonSRX horizontalConveyor{18};
  TalonSRX intakeMotor{21};
  TalonSRX C1Motor{10};
  TalonSRX C2Motor{11};

  //pidcontroller  
  rev::SparkMaxPIDController testPIDController = flywheelShooter1.GetPIDController();

  //controller
  frc::Joystick gamepad1{0};
  frc::Joystick gamepad2{1};

  //motors
  rev::CANSparkMax rightFront {10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightBack {11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftFront {12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftBack {13, rev::CANSparkMax::MotorType::kBrushless};  
  rev::CANSparkMax flywheelShooter1 {16, rev::CANSparkMax::MotorType::kBrushless};  

  //follows flywheelShooter1
  rev::CANSparkMax flywheelShooter2 {17, rev::CANSparkMax::MotorType::kBrushless};

  //encoders
  rev::SparkMaxRelativeEncoder leftEncoder = leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder rightEncoder = rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder flywheelEncoder = flywheelShooter1.GetEncoder();

  //Switch declarations
  frc::DigitalInput horizontalSwitch{1}; //port will change 
  frc::DigitalInput verticalSwitch{2}; //port will change

  //solenoids
  frc::DoubleSolenoid rightSolenoid{9, frc::PneumaticsModuleType::REVPH, 0, 1};
  frc::DoubleSolenoid leftSolenoid{9, frc::PneumaticsModuleType::REVPH, 2, 3};

  //constant variables
  const double wheelDiameterInches = 453453453; //TBD
  const double wheelCircumference = M_PI * wheelDiameterInches;
  const double countsPerRev = 42;
  const double gearReduction = 2323232; //TBD
  const double countsPerInch = (countsPerRev * gearReduction)/wheelCircumference;
  const double degrees;
  const double robotRadius = 2482304; //TBD
  const double degreesToInches = (degrees * (M_PI / 180) * robotRadius);

  //Limelight variables
  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double targetArea;
  double targetSkew;

  //angle to distance function variables:
  double trueAngle;
  double distanceToHub;
  double firstRun, secondRun, thirdRun;

  //PID
  //kP = 0.000001, kFF 0.0001695
  //Change the feed forward to adjust settling below or above the set point
  //Tuning a PID for future reference:  
  //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-pid-controller.html
  double kP = 0.000001, kFF = 0.00017375, kMaxoutput = 1, kminoutput = -1, kI = 1e-4, kD = 1;
  double motorVelocity = 0;
  double trueVelocity;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};