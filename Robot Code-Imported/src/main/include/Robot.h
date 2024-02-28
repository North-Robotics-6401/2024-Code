// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/motorcontrol/Spark.h>
#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
#include <rev/CANEncoder.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <iostream>
#include <frc/controller/PIDController.h>
 
//#include <frc/XboxController.h>
// aaron waz aqui!
//limelight imports
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "span.h"
#include "cameraserver/CameraServer.h"

#include <iostream>
#include <time.h>

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

		void SwerveDrive(
			double driveX, 
			double driveY, 
			double rotation, 
			double gyro,
			rev::CANSparkMax driveMotor, 
			rev::CANSparkMax spinMotor, 
			double driveYMultiplier,
			double driveXMultiplier,
			ctre::phoenix6::hardware::CANcoder CANCoder,
			frc::PIDController PIDController,
			double maxDrivePower,
			double maxSpinPower
		);

 private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;


	//*********SWERVE STUFF***********

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

	double driveX;
	double driveY;
	double rotation;
	double gyro;

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

	double spinmax = .3;
	double drivemax = .4;

	double lastFLSpeed = INFINITY;

	//*********OTHER STUFF***********

 	frc::Joystick leftJoyStick{0};
	frc::Joystick rightJoyStick{1};
 	frc::Joystick operatorJoyStick{2};
/*
	rev::CANSparkMax intakeMouth{9, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax intakeDeploy{10, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax shooter1{11, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax shooter2{12, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax climber1{13, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax climber2{14, rev::CANSparkMax::MotorType::kBrushless};
	*/

 
	const double shooter1ShootSetting = 0.5; //the setting to turn the shooter1 to while shooting (Default: 0.5)
	const double shooter2ShootSetting = -0.5; //the setting to turn the shooter2 to while shooting (Default: -0.5)
	const double shooter1RestSetting = 0; //the setting to turn the shooter2 to while resting (Default: 0) 
	const double shooter2RestSetting = 0; //the setting to turn the shooter2 to while resting (Default: 0)
	const double operatorJoyStickYDeadZone = 0.05; //the deadzone for the operator JoyStick on the Y Axis (Default: 0.05)

};

//Hello! This is a test comment made from Dylan's computer to test github commits!
//It worked. :)