// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>
#include <frc/AnalogGyro.h>

// Config vars
// bool m_is2022Robot = true;
// bool m_balancing_mode = false;

// Motors
frc::PWMSparkMax m_Spark_left{0};
frc::PWMSparkMax m_Spark_right{8};

// frc::VictorSP m_VictorSP_left{0}; 
// frc::VictorSP m_VictorSP_right{8};

// Encoders
frc::Encoder m_encoder_left{8,9, true};
frc::Encoder m_encoder_right{6,7, false};

// Gyro
frc::AnalogGyro m_gyro{0}; //check channel number

// Drive Config
frc::DifferentialDrive m_drive_system = frc::DifferentialDrive{m_Spark_left,m_Spark_right};
// frc::DifferentialDrive m_drive_system = frc::DifferentialDrive{m_VictorSP_left,m_VictorSP_right};

// Xbox Controller
frc::XboxController m_xbox{0};

void Robot::RobotInit() {
  // Debug encoder to dashboard
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Left Encoder", &m_encoder_left);
  frc::SmartDashboard::PutData("Right Encoder", &m_encoder_right);

  m_gyro.Calibrate();

  // if(m_is2022Robot)
  // {
  //  m_drive_system = frc::DifferentialDrive(m_Spark_left,m_Spark_right);
  // }
  // else
  // {    
  //   m_drive_system = frc::DifferentialDrive(m_VictorSP_left,m_VictorSP_right);
  // }


}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
}

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
  // Mode debug
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  // fmt::print("Auto selected: {}\n", m_autoSelected);

  // Init encoders
  m_encoder_left.Reset();
  m_encoder_right.Reset();
  m_encoder_left.SetDistancePerPulse(0.0521); // distance per pulse in inches
  m_encoder_right.SetDistancePerPulse(0.0521); // distance per pulse in inches
}

void Robot::AutonomousPeriodic() {
  // TODO: place game piece

  // IF BALANCING
  // if (m_balancing_mode == true)
  // {
  //   double robotPitchAngle = m_gyro.GetAngle();
  //   // if gyro is positive drive forward at slow speed (30%) until angle is zero
  //   if(robotPitchAngle > 0)
  //   {
  //     m_drive_system.TankDrive(-0.4, 0.4);
  //   }
  //   // if gyro is negative drive backwards at slow speed
  //   else if (robotPitchAngle < 0)
  //   {
  //     m_drive_system.TankDrive(-0.4, 0.4);
  //   }
  //   // if gyro is level STOP MOTORS
  //   else
  //   {
  //     m_drive_system.TankDrive(0, 0);
  //   }
  // }
  
  // Drive toward charging station while less than set distance
  if(m_encoder_left.GetDistance() < 200 && m_encoder_right.GetDistance() < 200)
  {
    // // Check gyro angle greater than 9 degrees
    // if(m_gyro.GetAngle() > 9)
    // {
    //   // If gyro angle is  then start balancing == set a flag
    //   m_balancing_mode = true; 
    //   m_drive_system.TankDrive(0.7,-0.7);
    // }
    // else 
    // {
      // If not keep driving
      m_drive_system.TankDrive(0.5,-0.5);
    }
  else
  {
    // Stop driving
    m_drive_system.TankDrive(0, 0);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  double m_maxspeed;
  double m_left_motorspeed, m_right_motorspeed;

  // Boost mode
  if (m_xbox.GetRightBumper()){
    m_maxspeed = 0.7;
  }
  else {
    m_maxspeed = 0.5;
  }

  m_left_motorspeed = m_xbox.GetLeftY()*-m_maxspeed;
  m_right_motorspeed = m_xbox.GetRightY()*(m_maxspeed);

  m_drive_system.TankDrive(m_left_motorspeed, m_right_motorspeed);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
