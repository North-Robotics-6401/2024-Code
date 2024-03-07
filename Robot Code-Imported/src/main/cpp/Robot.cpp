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
	SwerveModuleState moduleState,
	//double driveXMultiplier,
	//double driveYMultiplier,
	ctre::phoenix6::hardware::CANcoder* CANCoder,
	frc::PIDController* PIDController,
	int drivePowerNegation
	)
{

	/*
	/////////////////////     PREVIOUS SWERVE DRIVE CODE     ////////////////////
	/////////////////////     PREVIOUS SWERVE DRIVE CODE     ////////////////////
	/////////////////////     PREVIOUS SWERVE DRIVE CODE     ////////////////////

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
	*/


/*
	////////////// NEW SWERVE DRIVE CODE (USES SWERVEDRIVEKINEMATICS CLASS) ///////////////////////
*/
	double currentAngle = fmod(CANCoder->GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	moduleState.Optimize(moduleState, Rotation2d{units::radian_t{currentAngle}});

	driveMotor->Set(moduleState.speed.value() * drivePowerNegation);
	double spinMotorSpeed = PIDController->Calculate(moduleState.angle.Radians().value(), currentAngle);
	spinMotor->Set(spinMotorSpeed);
}

void Robot::RobotInit()
{
	m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {}

bool Robot::autoTimeBetween(double min, double max){
	return (autoSecondsElapsed >= min && autoSecondsElapsed <= max);
}

void Robot::AutonomousInit()
{
	gyro.ZeroYaw();
	autoSecondsElapsed = 0;
	timeNow = clock();
}

void Robot::AutonomousPeriodic(){
	
	//TIME STUFF ***
	autoSecondsElapsed += (double) ((clock() - timeNow)/((double) CLOCKS_PER_SEC)) * 5.0;
	timeNow = clock();

	//Control
	std::cout << autoSecondsElapsed << "\n";
	if(autoTimeBetween(0, 4)){
		driveX = 0;
		driveY = -.5;
		rotation = 0;
		std::cout<<"heeeeee\n";
	} else {
		driveX = 0;
		driveY = 0;
		rotation = 0;
	}


	//Driving
	double yaw = gyro.GetYaw();
	if(yaw < 0){
		yaw += 360.0;
	}
	yaw *= M_PI/180.0;

	yaw = 0;	//Remove this when testing whether the gyro works or not when driving.

	double inputs[] = {
		-driveX,
		driveY,
		rotation,
		yaw,
		0.3,
		0.5,
	};

	double movementAngle = atan2(driveY, driveX);
	double movementMagnitude = sqrt(driveX * driveX + driveY * driveY);
	ChassisSpeeds chassisSpeed = ChassisSpeeds::FromFieldRelativeSpeeds(
		units::velocity::meters_per_second_t{maxDriveSpeed * cos(movementAngle) * movementMagnitude},
		units::velocity::meters_per_second_t{maxDriveSpeed * sin(movementAngle) * movementMagnitude},
		units::angular_velocity::radians_per_second_t{maxRotationSpeed * rotation},
		Rotation2d{units::radian_t{yaw}}
	);
	wpi::array<SwerveModuleState, 4> moduleStates = swerveKinematics.ToSwerveModuleStates(chassisSpeed);
	swerveKinematics.DesaturateWheelSpeeds(&moduleStates, units::velocity::meters_per_second_t{1});

	controlSwerveDrive(inputs, &fldrive, &flspin, moduleStates[0], &flCANcoder, &flPID, -1);
	controlSwerveDrive(inputs, &frdrive, &frspin, moduleStates[1], &frCANcoder, &frPID, -1);
	controlSwerveDrive(inputs, &bldrive, &blspin, moduleStates[2], &blCANcoder, &blPID, 1);
	controlSwerveDrive(inputs, &brdrive, &brspin, moduleStates[3], &brCANcoder, &brPID, 1);
}

void Robot::TeleopInit() {
	//This could potentially be bad. 
	//This should go in autoinit before an actual match.
	//gyro.ZeroYaw();

	arm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	climber1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	climber2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::TeleopPeriodic()
{
	driveX = -leftJoyStick.GetX();
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
				//YOU MUST ALSO CHANGE THIS IN AUTONOMOUS


	double inputs[] = {
		driveX,
		driveY,
		rotation,
		yaw,
		0.3, //Max drive power
		0.5, //Max spin power,
	};

	double movementAngle = atan2(driveY, driveX);
	double movementMagnitude = sqrt(driveX * driveX + driveY * driveY);
	ChassisSpeeds chassisSpeed = ChassisSpeeds::FromFieldRelativeSpeeds(
		units::velocity::meters_per_second_t{maxDriveSpeed * cos(movementAngle) * movementMagnitude},
		units::velocity::meters_per_second_t{maxDriveSpeed * sin(movementAngle) * movementMagnitude},
		units::angular_velocity::radians_per_second_t{maxRotationSpeed * rotation},
		Rotation2d{units::radian_t{yaw}}
	);
	wpi::array<SwerveModuleState, 4> moduleStates = swerveKinematics.ToSwerveModuleStates(chassisSpeed);
	swerveKinematics.DesaturateWheelSpeeds(&moduleStates, units::velocity::meters_per_second_t{1});

	controlSwerveDrive(inputs, &fldrive, &flspin, moduleStates[0], &flCANcoder, &flPID, -1);
	controlSwerveDrive(inputs, &frdrive, &frspin, moduleStates[1], &frCANcoder, &frPID, -1);
	controlSwerveDrive(inputs, &bldrive, &blspin, moduleStates[2], &blCANcoder, &blPID, 1);
	controlSwerveDrive(inputs, &brdrive, &brspin, moduleStates[3], &brCANcoder, &brPID, 1);


	//Move arm
	if (operatorJoyStick.GetRawButton(11)){
		currentArmTarget = armPositions::Shooter;
	}
	else if (operatorJoyStick.GetRawButton(9)){
		currentArmTarget = armPositions::Amp;
	}
	else if (operatorJoyStick.GetRawButton(7)){
		currentArmTarget = armPositions::Ground;
	}

	double armSpeed = 0;
	double currentArmPosition = armEncoder.Get().value();
	if(currentArmTarget == armPositions::Shooter){
		armSpeed = armPID.Calculate(0.005, currentArmPosition);
	}else if(currentArmTarget == armPositions::Amp){
		armSpeed = armPID.Calculate(0.219, currentArmPosition);
	}else if(currentArmTarget == armPositions::Ground){
		armSpeed = armPID.Calculate(0.579247, currentArmPosition);
	}
	armSpeed = std::clamp(armSpeed, -0.1, 0.1);
	armSpeed = 0; //Arm is fucked. Come back to this later.
	arm.Set(armSpeed);
	//std::cout << armSpeed << " // " << armEncoder.Get().value() << "\n";


	//Gobble & Spit
	if (operatorJoyStick.GetRawButton(3)){
		mouth.Set(.7);
	}else if (operatorJoyStick.GetRawButton(4)){
		mouth.Set(-.7);
	}else{
		mouth.Set(0);
	}
	
	//Fire
	if (operatorJoyStick.GetTrigger()){
		shooter1.Set(speakerSpeed);
		shooter2.Set(-speakerSpeed);
	}else if(operatorJoyStick.GetRawButton(5)){
		shooter1.Set(ampSpeed);
		shooter1.Set(-ampSpeed);
	}else{
		shooter1.Set(0);
		shooter2.Set(0);
	}


	//Climb
	if(operatorJoyStick.GetRawButton(8)){
		climber1.Set(-0.5);
		climber2.Set(-0.5);
	}else if(operatorJoyStick.GetRawButton(10) || operatorJoyStick.GetRawButton(12)){
		 if(operatorJoyStick.GetRawButton(10)){
			climber1.Set(0.2);
		 }
		 if(operatorJoyStick.GetRawButton(12)){
			climber2.Set(0.2);
		 }
	}else{
		climber1.Set(0);
		climber2.Set(0);
	}
	//Make climbers not move when below a certain height and above a certain height. 
	//Max retraction is too low and max extension is too high.

	
}

void Robot::DisabledInit() {
	arm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	climber1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	climber2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit(){
	arm.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
	climber1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
	climber2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::TestPeriodic(){
	std::cout << armEncoder.Get().value() << " // " << climber1.GetEncoder(rev::SparkRelativeEncoder::EncoderType::kHallSensor, 1).GetPosition() << "\n";
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
	/*
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
*/
	
}


#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
