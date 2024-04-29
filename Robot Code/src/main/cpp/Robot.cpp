// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <cmath>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::controlSwerveModule(
	double inputs[],
	rev::CANSparkMax* driveMotor,
	rev::CANSparkMax* spinMotor,
	SwerveModuleState moduleState,
	ctre::phoenix6::hardware::CANcoder* CANCoder,
	frc::PIDController* PIDController,
	int drivePowerNegation
	)
{
	double currentAngle = fmod(CANCoder->GetPosition().GetValueAsDouble(), 1.0) * 2 * M_PI;
	moduleState = moduleState.Optimize(moduleState, Rotation2d{units::radian_t{currentAngle}});

	driveMotor->Set(moduleState.speed.value() * drivePowerNegation);
	double spinMotorSpeed = PIDController->Calculate(moduleState.angle.Radians().value(), currentAngle);
	spinMotor->Set(spinMotorSpeed);
}

void Robot::controlSwerveDrive(double movementX, double movementY, double rotationSpeed){
	//Gyro reads from -180 to 180 degrees. Converts into radians.
	double yaw = gyro.GetYaw();
	yaw *= M_PI/180.0;
	//yaw = 0;	//Remove this when testing whether the gyro works or not when driving.
	double inputs[] = {
		-movementX,
		movementY,
		rotationSpeed,
		yaw,
		0.3,
		0.5,
	};
	//Get inputs for ToSwerveModuleStates()
	double movementAngle = atan2(movementY, movementX);
	double movementMagnitude = sqrt(movementX * movementX + movementY * driveY);
	ChassisSpeeds chassisSpeed = ChassisSpeeds::FromFieldRelativeSpeeds(
		units::velocity::meters_per_second_t{maxDriveSpeed * cos(movementAngle) * movementMagnitude},
		units::velocity::meters_per_second_t{maxDriveSpeed * sin(movementAngle) * movementMagnitude},
		units::angular_velocity::radians_per_second_t{maxRotationSpeed * rotationSpeed},
		Rotation2d{units::radian_t{-yaw}}
	);
	//Get module states
	wpi::array<SwerveModuleState, 4> moduleStates = swerveKinematics.ToSwerveModuleStates(chassisSpeed);
	//Convert module state speeds to 0-1 scale
	swerveKinematics.DesaturateWheelSpeeds(&moduleStates, units::velocity::meters_per_second_t{1});
	//Control swerve motors using module states
	controlSwerveModule(inputs, &fldrive, &flspin, moduleStates[0], &flCANcoder, &flPID, -1);
	controlSwerveModule(inputs, &frdrive, &frspin, moduleStates[1], &frCANcoder, &frPID, -1);
	controlSwerveModule(inputs, &bldrive, &blspin, moduleStates[2], &blCANcoder, &blPID, 1);
	controlSwerveModule(inputs, &brdrive, &brspin, moduleStates[3], &brCANcoder, &brPID, 1);
}

void Robot::RobotInit()
{
	gyro.ZeroYaw();
	wheelToZero = 0;

	//Add auto options to smart dashboard
	m_chooser.SetDefaultOption(autoNames[0], autoNames[0]);
	for(int i = 1; i < (int) autoNames.size(); i ++){
		m_chooser.AddOption(autoNames[i], autoNames[i]);
	}
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {
	if(wheelToZero < 4){
		units::angle::turn_t newPos{0.0};
		units::time::second_t newTime{0.015};
		wheelToZero += 1;
		if(wheelToZero == 1){
			flCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 2){
			frCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 3){
			blCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 4){
			brCANcoder.SetPosition(newPos, newTime);	
		}else{
			std::cout << "Wheels zeroed!\n";
		}
	}
}

bool Robot::autoTimeBetween(double min, double max){
	return (autoSecondsElapsed >= min && autoSecondsElapsed <= max);
}

void Robot::AutonomousInit()
{
	autoSecondsElapsed = 0;
	timeNow = clock();
	std::cout << "Running " << m_chooser.GetSelected() << " auto!\n";
}

void Robot::AutonomousPeriodic(){
	
	//TIME STUFF
	autoSecondsElapsed += (double) ((clock() - timeNow)/((double) CLOCKS_PER_SEC)) * 5.0;
	timeNow = clock();

	//Auto driving code
	std::string selectedAutoMode = m_chooser.GetSelected();

	double shooterSpeed = 0;
	double mouthSpeed = 0;
	driveX = 0;
	driveY = 0;
	rotation = 0;

	if(selectedAutoMode == "ShootAndExit"){
		if(autoTimeBetween(0, 3)){
			shooterSpeed = speakerSpeed;
		}
		else if(autoTimeBetween(3,4)) {
			shooterSpeed = speakerSpeed;
			mouthSpeed = -0.7;
		}
		else if(autoTimeBetween(4,8)){
			driveY = 0.3;
		}
		//Normal auto code goes here
	}else if(selectedAutoMode == "JustShoot"){
		if(autoTimeBetween(0, 3)){
			shooterSpeed = speakerSpeed;
		}
		else if(autoTimeBetween(3,4)) {
			shooterSpeed = speakerSpeed;
			mouthSpeed = -0.7;
		}
	}else if(selectedAutoMode == "JustExit"){
		if(autoTimeBetween(0, 3)){
			driveY = 0.3;
		}
	}
	
	else if(selectedAutoMode == "Testing"){
		driveY = 0.05;
	}

	shooter1.Set(shooterSpeed);
	shooter2.Set(-shooterSpeed);
	mouth.Set(mouthSpeed);
	controlSwerveDrive(driveX, driveY, rotation);
}

void Robot::TeleopInit() {
	arm.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	climber1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	climber2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::TeleopPeriodic()
{
	driveX = leftJoyStick.GetX();
	driveY = -leftJoyStick.GetY();
	rotation = -rightJoyStick.GetX();

	controlSwerveDrive(driveX, driveY, rotation);
	bool driving = sqrt(driveX * driveX + driveY * driveY) > 0.5;
	//Move arm
	double armSpeed = 0;
	double currentArmPosition = armEncoder.Get().value();
	//Sometimes the arm will go too far forward or retracted and loop around to the negatives or >1.
	currentArmPosition = currentArmPosition < 0.0 ? currentArmPosition += 1.0 : currentArmPosition;
	currentArmPosition = currentArmPosition > 1.0 ? currentArmPosition -= 1.0 : currentArmPosition;
	double opJoystickY = operatorJoyStick.GetY();
	//This "registeredOpJoystickY" exists to soften sudden jerks of the joystick.
	registeredOpJoystickY = registeredOpJoystickY + ((opJoystickY - registeredOpJoystickY) * 0.2);
	//std::cout << currentArmPosition << "\n";
	bool pushingJoystick = opJoystickY < 0.105;
	bool pullingJoystick = opJoystickY > 0.105;
	shootingAmp = operatorJoyStick.GetRawButton(11) || (shootingAmp && !(pushingJoystick || pullingJoystick));
	if(!shootingAmp){
		armSpeed = registeredOpJoystickY * 0.5;
		//This slows the arm down as it approaches a target position.
		double forwardPercentage = std::clamp((currentArmPosition - armRetractedPosition)/(armForwardPosition - armRetractedPosition), 0.0, 1.0);
		armSpeed = pushingJoystick ? armSpeed * std::min(((-4 * (forwardPercentage - 0.75)) + 1), 1.0) : armSpeed;
		armSpeed = pullingJoystick ? armSpeed * std::min(((-4 * ((1 - forwardPercentage) - 0.75)) + 1), 1.0) : armSpeed;
		//Don't move the arm motor at all if we're too far forward or backward.
		bool tooFarForward = (pushingJoystick && currentArmPosition > armForwardPosition) || (currentArmPosition < 0.0);
		bool tooFarRetracted = (pullingJoystick && currentArmPosition < armRetractedPosition) && (currentArmPosition > 0.0); //Sometimes encoder will be negative when too far forward
		if((tooFarForward || tooFarRetracted) && !operatorJoyStick.GetRawButton(7)){
			armSpeed = 0;
		}
		//std::cout << forwardPercentage << " // " << armSpeed << " // " << currentArmPosition << " // " << tooFarForward << " // " << tooFarRetracted << "\n";
	}else{
		armSpeed = std::clamp(armPID.Calculate(armAmpPosition, currentArmPosition), -0.4, 0.4);
		//std::cout << currentArmPosition << " // " << armSpeed << "\n";
	}
	//Arm position 0.5267 is good for amp shooting
	arm.Set(armSpeed);
	

	//Gobble & Spit
	if (operatorJoyStick.GetRawButton(3)){
		mouth.Set(.7);
	}else if (operatorJoyStick.GetRawButton(4)){
		mouth.Set(-.7);	
	}else{
		mouth.Set(0);
	}
	
	//Fire
	double shooterSpeed = 0;
	if (operatorJoyStick.GetTrigger()){
		shooterSpeed = speakerSpeed;
	}else if(operatorJoyStick.GetRawButton(5)){
		shooterSpeed = ampSpeed;
	}
	if(driving){shooterSpeed *= 0.4;}
	shooter1.Set(shooterSpeed);
	shooter2.Set(-shooterSpeed);


	//Climb
	climber1.Set(0);
	climber2.Set(0);
	//std::cout << climber1Encoder.GetPosition() << " // " << climber2Encoder.GetPosition() << "\n";
	if(operatorJoyStick.GetRawButton(8)){
	//	if(climber1Encoder.GetPosition() > -110.0){
			climber1.Set(0.8);
	//	}
	//	if(climber2Encoder.GetPosition() > -110.0){
			climber2.Set(0.8);
	//	}
	}else{
		 if(operatorJoyStick.GetRawButton(10)/* && climber1Encoder.GetPosition() < 95.0*/){
			climber1.Set(-0.3);
		 }
		 if(operatorJoyStick.GetRawButton(12) /*&& climber2Encoder.GetPosition() < 85.0*/){
			climber2.Set(-0.3);
		 }
	}

	//Allow for zeroing of the field oriented drive through teleop.
	//If neccessary
	if(leftJoyStick.GetRawButtonPressed(5)){
		gyro.ZeroYaw();
	}
	
}

void Robot::DisabledInit() {
	arm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit(){
	bldrive.Set(0);
	brdrive.Set(0);
	frdrive.Set(0);
	fldrive.Set(0);
	blspin.Set(0);
	brspin.Set(0);
	frspin.Set(0);
	flspin.Set(0);
}

void Robot::TestPeriodic(){
	if(operatorJoyStick.GetTrigger() && wheelToZero <= 4){
		units::angle::turn_t newPos{0.0};
		units::time::second_t newTime{0.015};
		wheelToZero += 1;
		if(wheelToZero == 1){
			flCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 2){
			frCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 3){
			blCANcoder.SetPosition(newPos, newTime);
		}else if(wheelToZero == 4){
			brCANcoder.SetPosition(newPos, newTime);	
		}else{
			std::cout << "Wheels zeroed!\n";
		}
	}
	if(operatorJoyStick.GetRawButton(5)){
		wheelToZero = 0;
		//Wheels can not be zeroed continuously if the trigger is held down. Press the Fire.A button to allow for
		//zeroing the wheels again
	}
	climber1.Set(0);
	climber2.Set(0);
	if(operatorJoyStick.GetRawButton(8)){
		climber1.Set(-0.75);
		climber2.Set(-0.75);
	}else{
		 if(operatorJoyStick.GetRawButton(10)){
			climber1.Set(0.4);
		 }
		 if(operatorJoyStick.GetRawButton(12)){
			climber2.Set(0.4);
		 }
	}
	if(leftJoyStick.GetRawButtonPressed(5)){
		gyro.ZeroYaw();
	}
	//std::cout << armEncoder.Get().value() << " // " << climber1Encoder.GetPosition() << " // " << climber2Encoder.GetPosition() << "\n";

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
