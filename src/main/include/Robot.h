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
  
  //user defined functions
  double LimelightDistance();
  double GetMedian(double value1, double value2, double value3);
  void DistanceToRPM(double distance);
  void limelightAlign();
  void autoLimelightAlign();
  void shoot();
  void ShootemQuickie();
  void ScoringCargo();
  void lowerPortShot();
  void kentController();
  void joshController();
  void troyAndMichaelController();

  void encoderDrive(double speed, double leftInches, double rightInches, double timeoutSeconds);
  void horizontalConveyor(double speed, double rotations, double timeoutSeconds);
  void verticalConveyor(double speed, double rotations, double timeoutSeconds);
  void flywheel(double speed, double rotations, double timeoutSeconds);
  void turnDrive(double speed, double degrees, double timeoutSeconds);

<<<<<<< HEAD
  void calculateRotateValue(double distance, double speed);
  void microswitchIntake();

  int getPosition();

 
=======
  void drive(double distance, double speed);

  int getPosition();

 //CIMs  
  TalonSRX C2Motor{11};

  TalonSRX hConveyor{18};
  TalonSRX intake{21};
  TalonSRX vConveyorLeft{20};
  TalonSRX vConveyorRight{19};
>>>>>>> a4cae61e166ea7da92420c9a8bdea5bd9a1e2902
  //pidcontroller  
  rev::SparkMaxPIDController flywheelPID = flywheelShooter1.GetPIDController();
  rev::SparkMaxPIDController rightDrive = rightFront.GetPIDController();
  rev::SparkMaxPIDController leftDrive = leftFront.GetPIDController();

  

  //controller
  frc::Joystick gamepad1{0};
  frc::Joystick gamepad2{1};
  frc::Joystick Attack31{2};
  frc::Joystick Attack32{3};

  //gyro
  frc::ADXRS450_Gyro gyro;
  //motors
  rev::CANSparkMax rightFront {10, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rightBack {11, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftFront {12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax leftBack {13, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax hConveyor {18, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax vConveyorLeft {20, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax vConveyorRight {19, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax intake {21, rev::CANSparkMax::MotorType::kBrushless}; 

  rev::CANSparkMax flywheelShooter1 {16, rev::CANSparkMax::MotorType::kBrushless};  
  //follows flywheelShooter1
  rev::CANSparkMax flywheelShooter2 {17, rev::CANSparkMax::MotorType::kBrushless};

  //encoders
  rev::SparkMaxRelativeEncoder leftEncoder = leftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder rightEncoder = rightFront.GetEncoder();
  rev::SparkMaxRelativeEncoder flywheelEncoder = flywheelShooter1.GetEncoder();


  //micro-switches
  frc::DigitalInput horizontalSwitch{1};//port will change 
  frc::DigitalInput verticalSwitch{2}; //port will change

  //solenoids
  //  frc::DoubleSolenoid rightSolenoid {9, frc::PneumaticsModuleType::REVPH, 0, 1};
  //  frc::DoubleSolenoid leftSolenoid {9, frc::PneumaticsModuleType::REVPH, 2, 3};
  
  //constant variables
  const double wheelDiameterInches = 4; //TBD
  const double wheelCircumference = M_PI * wheelDiameterInches;
  const double countsPerRev = 42;
  const double gearReduction = 8.6; //TBD
  const double countsPerInch = (countsPerRev * gearReduction)/wheelCircumference;
  const double degrees = 0;
  const double robotRadius = 25; //TBD
  const double degreesToInches = (degrees * (M_PI / 180) * robotRadius);

  bool hasRun = false;
  double distance;
  //Switch declarations
  
  
  //score cargo method

  //frc::Timer *pTimer = new frc::Timer();
  
<<<<<<< HEAD
  //timer
  
=======

  units::time::second_t previousTime; 
  units::time::second_t alignPreviousTime;
  units::time::second_t elapsedTime;
  units::time::second_t alignElapsedTime;
  units::time::second_t startTime;
>>>>>>> a4cae61e166ea7da92420c9a8bdea5bd9a1e2902

  double gyroAngle; 
  double& error = gyroAngle;
  double speedFactor = error * 0.65; //arbitrary number (to be tested)
  double targetDistance; 
  double fractionOfTargetDistance;

  bool functionCompleted;
  bool alignmentComplete;

  double averageActualDistance;
  double speedChange; 
  //Gamepad assignments:
  //Controller 1
  double gamepad1_LStick = gamepad1.GetRawAxis(1);
  double gamepad1_RStick = gamepad1.GetRawAxis(5);
  double gamepad1_RTrigger = gamepad1.GetRawAxis(3);
  //Controller 2
  bool gamepad2_YButton = gamepad2.GetRawButtonPressed(4);
  bool gamepad2_XButton = gamepad2.GetRawButtonPressed(3);
  bool gamepad2_AButton = gamepad2.GetRawButtonPressed(1);
  double gamepad2_LTrigger = gamepad2.GetRawAxis(2);
  double gamepad2_RTrigger = gamepad2.GetRawAxis(3);

  //Limelight variables
  double targetOffsetAngle_Horizontal;
  double targetOffsetAngle_Vertical;
  double targetArea;
  double targetSkew;
  
  //angle to distance function variables:
  double trueAngle;
  double distanceToHub;
  
  double firstRun, secondRun, thirdRun;

  //kP = 0.000001, kFF 0.0001695
  //Change the feed forward to adjust settling below or above the set point
  //Tuning a PID for future reference:  
  //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-pid-controller.html
  //Decrease feed forward to decrease settling point, increase to increase settling point
  double kP = 0.000001, kFF = 0.00015283, kMaxoutput = 1, kminoutput = -1, kI = 1e-4, kD = 1;
  double motorVelocity = 0;
  double trueVelocity;

  bool encoderDriveRunning = true;

  bool isActive;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};