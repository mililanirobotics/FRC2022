#pragma once

#include <string>

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
#include <frc/PneumaticHub.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

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

//timer
#include <frc/Timer.h>



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
  
  // User Defined Functions
  double LimelightDistance();
  double DistanceToRPM(double distance);

  int getPosition();

  void limelightAlign();
  void autoLimelightAlign();
  void shoot();
  void lowerPortShot();
  void kentController();
  void tankDrive();
  void microswitchIntake();
  void solenoidExtension();
  void drive(double distance, double speed);

  
  //=========================================================================== 
  // Object declarations
  //===========================================================================


  //pidcontroller  
  rev::SparkMaxPIDController flywheelPID {flywheelShooter1.GetPIDController()};
  rev::SparkMaxPIDController rightDrive {rightFront.GetPIDController()};
  rev::SparkMaxPIDController leftDrive {leftFront.GetPIDController()};

  //controller
  frc::Joystick gamepad1{0};
  frc::Joystick gamepad2{1};
  frc::Joystick Attack31{2};
  frc::Joystick Attack32{3};

  //gyro
  frc::ADXRS450_Gyro gyro;

  //Drive motors
  rev::CANSparkMax rightFront {27, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightBack {11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftFront {12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftBack {13, rev::CANSparkMax::MotorType::kBrushless}; 
  
  //Vertical conveyor motors
  rev::CANSparkMax vConveyorLeft {20, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax vConveyorRight {19, rev::CANSparkMax::MotorType::kBrushless}; 
  
  //Intake motor
  rev::CANSparkMax intake {30, rev::CANSparkMax::MotorType::kBrushless}; 
  
  //Horizontal conveyor motor
  rev::CANSparkMax hConveyor {18, rev::CANSparkMax::MotorType::kBrushless}; 

  //Flywheel motors
  rev::CANSparkMax flywheelShooter1 {16, rev::CANSparkMax::MotorType::kBrushless};  
  rev::CANSparkMax flywheelShooter2 {17, rev::CANSparkMax::MotorType::kBrushless};

  //encoders
  rev::SparkMaxRelativeEncoder leftEncoder {leftFront.GetEncoder()};
  rev::SparkMaxRelativeEncoder rightEncoder {rightFront.GetEncoder()};
  rev::SparkMaxRelativeEncoder flywheelEncoder {flywheelShooter1.GetEncoder()};

  //micro-switches
  frc::DigitalInput horizontalSwitch {0};//port will change 
  frc::DigitalInput verticalSwitch {1}; //port will change

  //solenoids
  frc::PneumaticHub pneumaticHub {2};
  frc::DoubleSolenoid rightSolenoid {pneumaticHub.MakeDoubleSolenoid(0, 1)};
  frc::DoubleSolenoid leftSolenoid {pneumaticHub.MakeDoubleSolenoid(2, 3)};  
  
  //=========================================================================== 
  // Variable declarations
  //===========================================================================

  // Constant variables
  const double wheelDiameterInches {4}; 
  const double wheelCircumference {M_PI * wheelDiameterInches};
  const double countsPerRev {42};
  const double gearReduction {8.6}; 
  const double countsPerInch {(countsPerRev * gearReduction)/wheelCircumference};
  
  double& error {gyroAngle};
  double speedFactor {error * 0.65}; //arbitrary number (to be tested)
  double distance {0};
  double targetDistance {0}; 
  double fractionOfTargetDistance {0};
  double averageActualDistance {0};
  double gyroAngle {0}; 

  //angle to distance function variables:
  double trueAngle {0};
  double distanceToHub {0};
  

  /* Limelight variables
   *  
   * Change the feed forward to adjust settling below or above the set point
   * Tuning a PID for future reference:  
   * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-pid-controller.html
   * Decrease feed forward to decrease settling point, increase to increase settling point
   */

  double kP {0.000001}, kFF {0.000178}, kMaxoutput {1}, kminoutput {-1}, kI {1e-4}, kD {1};
  double motorVelocity {0};
  double trueVelocity;
  
  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double targetArea;
  double targetSkew;
  
  bool encoderDriveRunning {true};
  bool hasRun {false};
  bool functionCompleted;
  bool alignmentComplete;
  bool isActive;
  bool isShooting {false};
  bool leftDriveRunning {true};
  bool rightDriveRunning {true};

  //Variables for use of the timer within autonomous
  frc::Timer *pTimer = new frc::Timer();

  units::time::second_t previousTime {0}; 
  units::time::second_t alignPreviousTime {0};
  units::time::second_t elapsedTime {0};
  units::time::second_t alignElapsedTime {0};
  units::time::second_t startTime {0};

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "60 inches";
  const std::string kAutoNameCustom2 = "80 inches";
  const std::string kAutoNameCustom3 = "100 inches";
  
  std::string m_autoSelected;
};