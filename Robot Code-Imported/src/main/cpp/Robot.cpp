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

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  
}

void Robot::TestPeriodic() {

  double desiredangle = atan(leftJoyStick.GetY()/leftJoyStick.GetX());
  double currentangle = 0;//encoder.GetPosition();

  if((currentangle*2*M_PI - M_PI)/2 < desiredangle-.2){
    flspin.Set(.1);
    frspin.Set(.1);
    blspin.Set(.1);
    brspin.Set(.1);
  } else if ((currentangle*2*M_PI - M_PI)/2 > desiredangle+.2){
    flspin.Set(-.1);
    frspin.Set(-.1);
    blspin.Set(-.1);
    brspin.Set(-.1);
  } else {
    flspin.Set(0);
    frspin.Set(0);
    blspin.Set(0);
    brspin.Set(0);
  }

  if(rightJoyStick.GetY() > .1){
    fldrive.Set(.1);
    frdrive.Set(.1);
    bldrive.Set(.1);
    brdrive.Set(.1);
  } else if (rightJoyStick.GetY() < -.1){
    fldrive.Set(-.1);
    frdrive.Set(-.1);
    bldrive.Set(-.1);
    brdrive.Set(-.1);
  } else {
    fldrive.Set(0);
    frdrive.Set(0);
    bldrive.Set(0);
    brdrive.Set(0);
  }

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

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
