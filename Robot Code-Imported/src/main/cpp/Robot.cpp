// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cmath>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::controlSwerveDrive(
	double inputs[],
	rev::CANSparkMax* driveMotor,
	rev::CANSparkMax* spinMotor,
	double driveYMultiplier,
	double driveXMultiplier,
	ctre::phoenix6::hardware::CANcoder* CANCoder,
	frc::PIDController* PIDController
	)
{
	double input_driveX = -inputs[0];
	double input_driveY = inputs[1];
	double input_rotation = inputs[2];
	double yaw = inputs[3];
	double maxDrivePower = inputs[4];
	double maxSpinPower = inputs[5];

	double desiredRotationAngle = atan2(input_driveY - input_rotation * driveYMultiplier, input_driveX + input_rotation * driveXMultiplier) - yaw;
	if (desiredRotationAngle < 0)
		desiredRotationAngle += 2 * M_PI;
	

	double currentAngle = fmod(CANCoder->GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;

	// all values beyond this point are  0 < x < 2pi
	// esure theyre within pi rad of eachother
	if (desiredRotationAngle - currentAngle > M_PI)
		currentAngle += 2 * M_PI;
	else if (currentAngle - desiredRotationAngle > M_PI)
		currentAngle -= 2 * M_PI;

	// ensure theyre within pi/2 rad of eachother
	double powerflip = -1; //flips the power if the second optmization is used
	if (desiredRotationAngle - currentAngle > M_PI / 2)
		desiredRotationAngle -= M_PI;
	else if (desiredRotationAngle - currentAngle < -M_PI / 2)
		desiredRotationAngle += M_PI;
	else
		powerflip = 1;


	//(each wheel is 17.5in from the geometric center of the robot)
	double drivePower = (powerflip * std::clamp(sqrt(pow(input_driveX, 2) + pow(input_driveY, 2)) + input_rotation * 17.5 * sin(desiredRotationAngle), -maxDrivePower, maxDrivePower));
	driveMotor->Set(drivePower);

	//TODO fix wheel spinning the wrong way while rotating

	//**********PID THINGS*************
	double spinMotorSpeed = std::clamp(PIDController->Calculate(desiredRotationAngle, currentAngle), -maxSpinPower, maxSpinPower);
	spinMotor->Set(spinMotorSpeed); //multiply by sign of the rotation controller X
}


void Robot::RobotInit()
{
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
void Robot::AutonomousInit()
{
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString("Auto Selector",
	//     kAutoNameDefault);
	fmt::print("Auto selected: {}\n", m_autoSelected);

	if (m_autoSelected == kAutoNameCustom)
	{
		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
}

void Robot::AutonomousPeriodic()
{
	if (m_autoSelected == kAutoNameCustom)
	{

		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
}

void Robot::TeleopInit() {
	//This could potentially be bad. 
	//This should go in autoinit before an actual match.
	gyro.ZeroYaw();
}

void Robot::TeleopPeriodic()
{
	driveX = leftJoyStick.GetX();
	driveY = leftJoyStick.GetY();
	rotation = rightJoyStick.GetX();

	//Gyro reads from -180 to 180 degrees.
	//This code converts from 0 to 360, then into radians.
	double yaw = gyro.GetYaw();
	if(yaw < 0){
		yaw += 360.0;
	}
	yaw *= M_PI/180.0;

	yaw = 0;	//Remove this when testing whether the gyro works or not when driving.
				//Also the gyro is zeroed in teleopinit.


	double inputs[] = {
		driveX,
		driveY,
		rotation,
		yaw,
		0.3, //Max drive power
		0.5, //Max spin power
	};

	/*
	double movementAngle = atan2(driveY, driveX);
	double movementMagnitude = sqrt(driveX * driveX + driveY * driveY);
	ChassisSpeeds chassisSpeed = ChassisSpeeds::FromFieldRelativeSpeeds(
		maxDriveSpeed * cos(movementAngle) * movementMagnitude,
		maxDriveSpeed * sin(movementAngle) * movementMagnitude,
		maxRotationSpeed * rotation,
		Rotation2d{units::radian_t{yaw}}
	);
	wpi::array<SwerveModuleState, 4> moduleStates = swerveKinematics.ToSwerveModuleStates(chassisSpeed);
	*/

	//Swerve drive code now calls a function. Old copy pasted code can be found in simulationperiodic. I tested it and this works fine.
	controlSwerveDrive(inputs, &fldrive, &flspin, 12.375, -12.375, &flCANcoder, &flPID);
	controlSwerveDrive(inputs, &frdrive, &frspin, 12.375, 12.375, &frCANcoder, &frPID);
	controlSwerveDrive(inputs, &bldrive, &blspin, -12.375, -12.375, &blCANcoder, &blPID);
	controlSwerveDrive(inputs, &brdrive, &brspin, -12.375, 12.375, &brCANcoder, &brPID);

	/***************REST OF THE CODE***************/

	/*
		//Arms disabled because for now since arm has unlimited range of movement.
		//They're installing an encoder on the arm motor though so we'll have to use that.
		double opJoyY = abs(operatorJoyStick.GetY()) > .05 ? -operatorJoyStick.GetY() : 0.0;
		if (abs(opJoyY) > 0.05) 
			arm.Set(opJoyY * -1);
	*/

	// arm.Set(abs(operatorJoyStick.GetY()) > .05 ? -operatorJoyStick.GetY() : 0.0); // alt way to do it with built-in deadzone

	

	if (operatorJoyStick.GetRawButton(3)){
		mouth.Set(.7);
	}else if (operatorJoyStick.GetRawButton(4)){
		mouth.Set(-.7);
	}else{
		mouth.Set(0);
	}
	
	if (operatorJoyStick.GetTrigger()){
		shooter1.Set(.5);
		shooter2.Set(-.5);
	}
	else{
		shooter1.Set(0);
		shooter2.Set(0);
	}

	
	
	


}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit(){
	gyro.ZeroYaw();
}

void Robot::TestPeriodic(){
	driveX = leftJoyStick.GetX();
	driveY = leftJoyStick.GetY();
	rotation = rightJoyStick.GetX();

	double yaw = 0; // FIGURE OUT FOR FIELD-CENTRIC ROTATION
	yaw = gyro.GetYaw();
	if(yaw < 0){
		yaw += 360.0;
	}
	yaw *= (M_PI/180.0);

	fldesired = atan2(driveY - rotation * 12.375, driveX + rotation * -12.375) - yaw;
	if (fldesired < 0)
	{
		fldesired += 2 * M_PI;
	}

	frdesired = atan2(driveY - rotation * 12.375, driveX + rotation * 12.375) - yaw;
	if (frdesired < 0)
	{
		frdesired += 2 * M_PI;
	}

	bldesired = atan2(driveY - rotation * -12.375, driveX + rotation * -12.375) - yaw;
	if (bldesired < 0)
	{
		bldesired += 2 * M_PI;
	}

	brdesired = atan2(driveY - rotation * -12.375, driveX + rotation * 12.375) - yaw;
	if (brdesired < 0)
	{
		brdesired += 2 * M_PI;
	}

	flcurrent = fmod(flCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	frcurrent = fmod(frCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	blcurrent = fmod(blCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	brcurrent = fmod(brCANcoder.GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;

	// all values beyond this point are  0 < x < 2pi

	// esure theyre within pi rad of eachother
	if (fldesired - flcurrent > M_PI)
		flcurrent += 2 * M_PI;
	else if (flcurrent - fldesired > M_PI)
		flcurrent -= 2 * M_PI;
	// ensure theyre within pi/2 rad of eachother
	if (fldesired - flcurrent > M_PI / 2)
		fldesired -= M_PI;
	else if (fldesired - flcurrent < -M_PI / 2)
		fldesired += M_PI;

	if (frdesired - frcurrent > M_PI)
		frcurrent += 2 * M_PI;
	else if (frcurrent - frdesired > M_PI)
		frcurrent -= 2 * M_PI;
	if (frdesired - frcurrent > M_PI / 2)
		frdesired -= M_PI;
	else if (frdesired - frcurrent < -M_PI / 2)
		frdesired += M_PI;

	if (bldesired - blcurrent > M_PI)
		blcurrent += 2 * M_PI;
	else if (blcurrent - bldesired > M_PI)
		blcurrent -= 2 * M_PI;
	if (bldesired - blcurrent > M_PI / 2)
		bldesired -= M_PI;
	else if (bldesired - blcurrent < -M_PI / 2)
		bldesired += M_PI;

	if (brdesired - brcurrent > M_PI)
		brcurrent += 2 * M_PI;
	else if (brcurrent - brdesired > M_PI)
		brcurrent -= 2 * M_PI;
	if (brdesired - brcurrent > M_PI / 2)
		brdesired -= M_PI;
	else if (brdesired - brcurrent < -M_PI / 2)
		brdesired += M_PI;

	// TODO VVVVVV
	// go thru our heads with test cases of all of the optimizations
	// add the negation of the wheel power if you use the second optimization.

	//(each wheel is 17.5in from the geometric center of the robot)
	flpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation * 17.5 * sin(fldesired), -drivemax, drivemax);
	frpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation * 17.5 * sin(frdesired), -drivemax, drivemax);
	blpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation * 17.5 * sin(bldesired), -drivemax, drivemax);
	brpower = std::clamp(sqrt(pow(driveX, 2) + pow(driveY, 2)) + rotation * 17.5 * sin(brdesired), -drivemax, drivemax);

	// set wheels to power
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

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
