// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cmath>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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

void Robot::AutonomousPeriodic() {
	if (m_autoSelected == kAutoNameCustom) {
	
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
	
	driveX = leftJoyStick.GetX();
	driveY = leftJoyStick.GetY();
	rotation = rightJoyStick.GetX();

	fldesired = atan2(driveY- rotation *  12.375, driveX + rotation * -12.375);
	frdesired = atan2(driveY- rotation *  12.375, driveX + rotation *  12.375);
	bldesired = atan2(driveY- rotation * -12.375, driveX + rotation * -12.375);
	brdesired = atan2(driveY- rotation * -12.375, driveX + rotation *  12.375);

	flcurrent = fmod(flCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	if(flcurrent > M_PI){flcurrent -= 2 * M_PI;}

	std::cout << flPID.Calculate(flcurrent, fldesired) << " // " << flcurrent << "\n";
	flspin.Set(flPID.Calculate(flcurrent, fldesired));
	
	//std::cout << flCANcoder.GetPosition().GetValueAsDouble() * 2 * M_PI << "\n";
	//std::cout << "test\n";

	//frspin.Set(.2);
	//std::cout << frCANcoder.GetPosition().GetValueAsDouble() * 2 * M_PI << "\n";
/*
	std::cout << blCANcoder.GetPosition().GetValueAsDouble() * 2 * M_PI << "\n";
	std::cout << brCANcoder.GetPosition().GetValueAsDouble() * 2 * M_PI << "\n";
*/
	/*
	std::cout << "desiredFL: " << fldesired << "\n";
	std::cout << "desiredFR: " << frdesired << "\n";
	std::cout << "desiredBL: " << bldesired << "\n";
	std::cout << "desiredBR: " << brdesired << "\n";
	std::cout << "\n\n";
	*/



	//create PIDs
	//good job ram family!

	//desiredFR = atan2(driveY,driveX);

	
	/*
	double desiredangle = atan(leftJoyStick.GetY()/leftJoyStick.GetX());
	double currentangle = 0;//encoder.GetPosition();


	if(operatorJoyStick.GetTrigger())
	{
		shooter1.Set(shooter1ShootSetting);
		shooter2.Set(shooter2ShootSetting);
	}

	else {
		shooter1.Set(shooter1RestSetting);
		shooter2.Set(shooter2RestSetting);
	}

	if (abs(operatorJoyStick.GetY()) > operatorJoyStickYDeadZone) {intakeDeploy.Set(operatorJoyStick.GetY() * -1);};

	//intakeDeploy.Set(operatorJoyStick.GetY() * -1);
	//intakeMouth.Set(operatorJoyStick.GetY() * -1);

	//spinMotor.Set(leftJoystick.GetRawAxis(0) * 0.2f); //0 is X axis
	//driveMotor.Set(rightJoystick.GetRawAxis(1) * 0.2f); //1 is Y axis
*/
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
	
}

void Robot::TestPeriodic() {

	//flspin.Set(rightJoyStick.GetX());
	//cancoderPosition = CANCoder1.GetPosition().GetValueAsDouble();
	//std::cout << cancoderPosition << "\n";
	
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
