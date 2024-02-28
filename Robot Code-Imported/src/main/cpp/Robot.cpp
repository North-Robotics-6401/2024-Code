// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cmath>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>


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
	)
	{
	double desiredAngle = atan2(driveY - rotation *  driveYMultiplier, driveX + rotation * driveXMultiplier) - gyro;
		if(desiredAngle < 0){desiredAngle += 2 * M_PI;}

	double currentAngle = fmod(CANCoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;

//all values beyond this point are  0 < x < 2pi

//esure theyre within pi rad of eachother
	if(desiredAngle - currentAngle > M_PI)
		currentAngle += 2*M_PI;
	else if(currentAngle - desiredAngle > M_PI)
		currentAngle -= 2*M_PI;

//ensure theyre within pi/2 rad of eachother
	if(desiredAngle - currentAngle > M_PI/2)
		desiredAngle -= M_PI;
	else if (desiredAngle - currentAngle < -M_PI/2)
		desiredAngle += M_PI;

	//(each wheel is 17.5in from the geometric center of the robot)
	double drivePower = std::min(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation*17.5*sin(desiredAngle), maxDrivePower);

	//set wheels to power
	driveMotor.Set(drivePower);

	//**********PID THINGS*************
	double spinMotorSpeed = std::min(PIDController.Calculate(desiredAngle, currentAngle), maxSpinPower);
	spinMotor.Set(spinMotorSpeed);
}


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
	
	gyro = 0; //FIGURE OUT FOR FIELD-CENTRIC ROTATION

	fldesired = atan2(driveY - rotation *  12.375, driveX + rotation * -12.375) - gyro;
		if(fldesired < 0){fldesired += 2 * M_PI;}

	frdesired = atan2(driveY - rotation *  12.375, driveX + rotation *  12.375) - gyro;
		if(frdesired < 0){frdesired += 2 * M_PI;}

	bldesired = atan2(driveY - rotation * -12.375, driveX + rotation * -12.375) - gyro;
		if(bldesired < 0){bldesired += 2 * M_PI;}

	brdesired = atan2(driveY - rotation * -12.375, driveX + rotation *  12.375) - gyro;
		if(brdesired < 0){brdesired += 2 * M_PI;}


	flcurrent = fmod(flCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	frcurrent = fmod(frCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	blcurrent = fmod(blCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	brcurrent = fmod(brCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;

//all values beyond this point are  0 < x < 2pi

//esure theyre within pi rad of eachother
	if(fldesired - flcurrent > M_PI)
		flcurrent += 2*M_PI;
	else if(flcurrent - fldesired > M_PI)
		flcurrent -= 2*M_PI;
//ensure theyre within pi/2 rad of eachother
	if(fldesired - flcurrent > M_PI/2)
		fldesired -= M_PI;
	else if (fldesired - flcurrent < -M_PI/2)
		fldesired += M_PI;

	if(frdesired - frcurrent > M_PI)
		frcurrent += 2*M_PI;
	else if(frcurrent - frdesired > M_PI)
		frcurrent -= 2*M_PI;
	if(frdesired - frcurrent > M_PI/2)
		frdesired -= M_PI;
	else if (frdesired - frcurrent < -M_PI/2)
		frdesired += M_PI;

	if(bldesired - blcurrent > M_PI)
		blcurrent += 2*M_PI;
	else if(blcurrent - bldesired > M_PI)
		blcurrent -= 2*M_PI;
	if(bldesired - blcurrent > M_PI/2)
		bldesired -= M_PI;
	else if (bldesired - blcurrent < -M_PI/2)
		bldesired += M_PI;

	if(brdesired - brcurrent > M_PI)
		brcurrent += 2*M_PI;
	else if(brcurrent - brdesired > M_PI)
		brcurrent -= 2*M_PI;
	if(brdesired - brcurrent > M_PI/2)
		brdesired -= M_PI;
	else if (brdesired - brcurrent < -M_PI/2)
		brdesired += M_PI;

		//TODO VVVVVV
		//go thru our heads with test cases of all of the optimizations 
		//add the negation of the wheel power if you use the second optimization.

	

	//(each wheel is 17.5in from the geometric center of the robot)
	flpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation*17.5*sin(fldesired), -drivemax, drivemax);
	frpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation*17.5*sin(frdesired), -drivemax, drivemax);
	blpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation*17.5*sin(bldesired), -drivemax, drivemax);
	brpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation*17.5*sin(brdesired), -drivemax, drivemax);

	std::cout << flpower << "\n";


	//set wheels to power
	fldrive.Set(flpower);
	frdrive.Set(frpower);
	bldrive.Set(blpower);
	brdrive.Set(brpower);
	


	//**********PID THINGS*************
	double flSpeed = std::min(flPID.Calculate(fldesired, flcurrent), spinmax);
	flspin.Set(flSpeed);

	double frSpeed = std::min(frPID.Calculate(frdesired, frcurrent), spinmax);
	frspin.Set(frSpeed);

	double blSpeed = std::min(blPID.Calculate(bldesired, blcurrent), spinmax);
	blspin.Set(blSpeed);

	double brSpeed = std::min(brPID.Calculate(brdesired, brcurrent), spinmax);
	brspin.Set(brSpeed);
	//

	/***************BACKUP IF PID DOESN'T WORK***************/
	 
	//TODO make artificial backup swerve


		//flspin.Set((fldesired - flcurrent) / M_PI);
		
		/*

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
	//std::cout << flCANcoder.GetPosition().GetValueAsDouble() << "\n";

	driveX = leftJoyStick.GetX();
	driveY = leftJoyStick.GetY();
	rotation = rightJoyStick.GetX();
	//SwerveDrive(driveX, driveY, rotation, 0.0, fldrive, flspin, 12.375, -12.375, flCANcoder, flPID, 0.3, 0.5);
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
