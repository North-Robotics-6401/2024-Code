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

 private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	rev::CANSparkMax flspin{1, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax blspin{3, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax frspin{5, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax brspin{7, rev::CANSparkMax::MotorType::kBrushless};

	rev::CANSparkMax fldrive{2, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax bldrive{4, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax brdrive{6, rev::CANSparkMax::MotorType::kBrushless};
	rev::CANSparkMax frdrive{8, rev::CANSparkMax::MotorType::kBrushless};


	
	frc::PIDController flPID{0.1, 0.01, 0.02}; 	
	//frc::PIDController frPID{0.1, 0.01, 0.02}; 	
	//frc::PIDController blPID{0.1, 0.01, 0.02}; 	
	//frc::PIDController brPID{0.1, 0.01, 0.02}; 


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

	double driveX = 0;
	double driveY = 0;
	double rotation = 0;

	double desiredFL =0;
	double desiredFR =0;
	double desiredBL =0;
	double desiredBR =0;


	//calculate the angel we want the wheels to be
	//during teleop 


	double cancoderPosition = 0;


	
ctre::phoenix6::hardware::CANcoder frEncoder{1};
ctre::phoenix6::hardware::CANcoder fLEncoder{2};
ctre::phoenix6::hardware::CANcoder blEncoder{3};
ctre::phoenix6::hardware::CANcoder brEncoder{4};
};

//Hello! This is a test comment made from Dylan's computer to test github commits!
//It worked. :)