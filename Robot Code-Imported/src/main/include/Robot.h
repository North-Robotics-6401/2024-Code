// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/Spark.h>
#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
#include <rev/CANEncoder.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <iostream>
#include <frc/controller/PIDController.h>
#include "AHRS.h"
#include <frc/SPI.h>
#include <wpi/array.h>
#include <frc/DutyCycleEncoder.h>
 
/*
//limelight imports
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "cameraserver/CameraServer.h"
*/

#include <iostream>
#include <time.h>

using namespace frc;

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
		void SimulationInit() override;
		void SimulationPeriodic() override;
		bool autoTimeBetween(double min, double max); // TIME FUNCTION
		void controlSwerveModule(
			double inputs[],
			rev::CANSparkMax* driveMotor,
			rev::CANSparkMax* spinMotor,
			SwerveModuleState moduleState,
			ctre::phoenix6::hardware::CANcoder* CANCoder,
			frc::PIDController* PIDController,
			int drivePowerNegation
		);
		void controlSwerveDrive(
			double movementX,
			double movementY,
			double rotationSpeed
		);

 private:
	frc::SendableChooser<std::string> m_chooser;
	std::string m_autoSelected;
	std::array<std::string, 5> autoNames = {"ShootAndExit", "JustShoot", "JustExit", "None", "Testing"};


	// *********SWERVE STUFF***********
	rev::CANSparkMax flspin{7, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax frspin{5, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax blspin{1, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax brspin{3, rev::CANSparkMax::MotorType::kBrushless};

	rev::CANSparkMax fldrive{8, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax frdrive{6, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax bldrive{2, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax brdrive{4, rev::CANSparkMax::MotorType::kBrushless};

	//kp,ki,kd
	frc::PIDController flPID{0.5, 0.035, .009};
	frc::PIDController frPID{0.5, 0.035, .009}; 	
	frc::PIDController blPID{0.5, 0.035, .009}; 
	frc::PIDController brPID{0.5, 0.035, .009}; 

	ctre::phoenix6::hardware::CANcoder flCANcoder{4};
	ctre::phoenix6::hardware::CANcoder frCANcoder{3};
	ctre::phoenix6::hardware::CANcoder blCANcoder{1};
	ctre::phoenix6::hardware::CANcoder brCANcoder{2};

	AHRS gyro{frc::SPI::Port::kMXP};
	
	double speakerSpeed = 1.0;
	double ampSpeed = 0.19;

	double driveX;
	double driveY;
	double rotation;

	double fldesired;
	double frdesired;
	double bldesired;
	double brdesired;

	double flcurrent;
	double frcurrent;
	double blcurrent;
	double brcurrent;

	double flpower;
	double frpower;
	double blpower;
	double brpower;

	double spinmax = .1;
	double drivemax = .4;


	// ** TIME VARIABLES **
	clock_t timeNow = 0; //Variable used in calculating change in time.
	double autoSecondsElapsed = 0.0; //Total amount of seconds that have elapsed since auto started.

	// *********OTHER STUFF***********

 	frc::Joystick leftJoyStick{0};
	frc::Joystick rightJoyStick{1};
 	frc::Joystick operatorJoyStick{2};
	//frc::POVButton

	units::meter_t distFromCenter{12.375/39.3701};	//12.375 inches converted to meters
	frc::SwerveDriveKinematics<4> swerveKinematics{
		Translation2d{-distFromCenter, distFromCenter},
		Translation2d{distFromCenter, distFromCenter},
		Translation2d{-distFromCenter, -distFromCenter},
		Translation2d{distFromCenter, -distFromCenter},
	};

	double maxDriveSpeed = 1.0;		//Values are in meters per second. ??????
	double maxRotationSpeed = 1.0;
	//Notes: Our gyro isn't in the center of our robot, so maybe that's causing issues.

	rev::CANSparkMax mouth{9, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax arm{10, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax shooter1{11, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax shooter2{12, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax climber1{13, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax climber2{14, rev::CANSparkMax::MotorType::kBrushless};

	frc::DutyCycleEncoder armEncoder{2};
	frc::PIDController armPID{1.5, 0.5, .25};

	rev::SparkRelativeEncoder climber1Encoder = climber1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);
	rev::SparkRelativeEncoder climber2Encoder = climber2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42);

	bool shootingAmp = false;
	double registeredOpJoystickY = 0;

	int encoderIndex = 0;
	bool zeroingDebounce = true;


	/*
	Arm ground position: 0.55
	Arm amp position: 0.219
	Arm shooting position: -0.0288
	*/

 /*
	const double shooter1ShootSetting = 0.5; //the setting to turn the shooter1 to while shooting (Default: 0.5)
	const double shooter2ShootSetting = -0.5; //the setting to turn the shooter2 to while shooting (Default: -0.5)
	const double shooter1RestSetting = 0; //the setting to turn the shooter2 to while resting (Default: 0) 
	const double shooter2RestSetting = 0; //the setting to turn the shooter2 to while resting (Default: 0)
	const double operatorJoyStickYDeadZone = 0.05; //the deadzone for the operator JoyStick on the Y Axis (Default: 0.05)
*/
};