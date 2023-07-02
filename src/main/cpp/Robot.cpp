// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Melbourne RoboCats #5648

#include "Robot.h"
#include <iostream> 

#include <fmt/core.h>
#include "cameraserver/CameraServer.h"
#include "cscore_oo.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Timer.h>
#include <frc/AddressableLED.h>

//LED includes

  static constexpr int kLength = 40;

  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{5};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;

#define BOOST_SPEED (0.9)
#define NORMAL_SPEED (0.7)
#define AUTO_BACK_DIST (-4500) //240 at 80% speed, 200 at 40%
#define AUTO_CHARGE_DIST (2200)
#define GO_FORWARD_COUNT (100)

void BalancingMode();

// Config vars
bool m_balancing_mode = false;

bool m_cone_deposit = false;

bool m_out_of_comunity = false;

double m_speedmod = 0;
int m_balanceStateFlag = 0; // Forward = 0, Balanced = 1, Backward = 2

int m_count = 0;

// Motors
frc::PWMSparkMax m_Spark_left{2};
frc::PWMSparkMax m_Spark_right{1};

// Encoders
frc::Encoder m_encoder_left{8,9, false};  //motor left side
frc::Encoder m_encoder_right{6,7, true};  //motor right side

// Drive Config
frc::DifferentialDrive m_drive_system = frc::DifferentialDrive{m_Spark_left,m_Spark_right};

// Xbox Controller (should be mapped to port 1)
frc::XboxController m_xbox{0};

// Camera
cs::UsbCamera m_camera_drive;

// Network tables
nt::NetworkTableEntry m_cameraSelection;


void Robot::RobotInit() {

  // Default to a length of 40, start empty output
  // Length is expensive to set, so only set it once, then just update data
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();
  for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetHSV(320, 255, 50);
    }
  // Set the LEDs
  m_led.SetData(m_ledBuffer);

  // Debug encoder to dashboard
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Left Encoder", &m_encoder_left);
  frc::SmartDashboard::PutData("Right Encoder", &m_encoder_right);
  frc::SmartDashboard::PutData("Drive System", &m_drive_system);

  m_encoder_left.SetDistancePerPulse(1.43); // distance per pulse in inches
  m_encoder_right.SetDistancePerPulse(1.43); // distance per pulse in inches


  //encoders for limits
  m_encoder_left.Reset();
  m_encoder_right.Reset();

  m_camera_drive = frc::CameraServer::StartAutomaticCapture("Drive Camera", 0);
  m_camera_drive.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

  //Check port mapping for joystick and gamepad
  if (m_xbox.GetType() != frc::GenericHID::kXInputGamepad)
  {
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "********** xbox controller mapped to wrong port ***********" << std::endl;
    std::cout << std::endl << std::endl << std::endl;

    //while(true);
    // FIXME - instead of infinite while loop, should throw an exception
  };
  
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
  
  //if(m_encoder_horizontal )
  /*
  If horiz. encoder is less than min extension 
  don't allow vert. encoder to fall lower than 20 cm above base.

  dont let horiz. encoder go past legal extension

  buffer on vert. encoder max

  claw buffers
  */

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
  //m_autoSelected = m_chooser.GetSelected();
  //m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  // fmt::print("Auto selected: {}\n", m_autoSelected);

  // Init encoders
  m_encoder_left.Reset();
  m_encoder_right.Reset();

  //Init counter
  m_count = 0;
}

void Robot::AutonomousPeriodic() {
  // place game piece
   if(m_cone_deposit == false)
    {

      if(m_count < GO_FORWARD_COUNT)
      {
        m_drive_system.TankDrive(0.5,-0.5);
        m_count = m_count + 1;
        }
      else{
        m_drive_system.TankDrive(0,0);
        m_cone_deposit = true;
          m_encoder_left.Reset();
          m_encoder_right.Reset(); //COMMENT OUT THIS LINE TO DISABLE MOVE BACKWARD AND BALANCE

      }
    }
    
    if(m_cone_deposit == true && m_out_of_comunity == false)
    {
      // Drive toward charging station while less than set distance
      if(m_encoder_left.GetDistance() <= AUTO_BACK_DIST || m_encoder_right.GetDistance() <= AUTO_BACK_DIST)
      {
        m_drive_system.TankDrive(0,0);
        m_out_of_comunity = true;
         m_encoder_left.Reset();
          m_encoder_right.Reset();
      }
      else{
        m_drive_system.TankDrive(-0.78,0.78); // previous .8
        
      }
    }
   /* if(m_out_of_comunity == true)
    {
    if(m_encoder_left.GetDistance() <= AUTO_CHARGE_DIST || m_encoder_right.GetDistance() <= AUTO_CHARGE_DIST)
    {
    m_drive_system.TankDrive(0.62,-0.62); //was 0.58
    }
    else
    {m_drive_system.TankDrive(0.0,0.0);}
     } */
  }

  

void Robot::TeleopInit() {
  m_balancing_mode = false;
}

void Robot::TeleopPeriodic() {
  double m_maxspeed;
  double m_left_motorspeed, m_right_motorspeed;

    // Boost mode
  if (m_xbox.GetRightBumper()){
    m_maxspeed = BOOST_SPEED;
    
  }
  else {
    m_maxspeed = NORMAL_SPEED;
    
  }

  m_left_motorspeed = m_xbox.GetLeftY()*-m_maxspeed;
  m_right_motorspeed = m_xbox.GetRightY()*m_maxspeed;

  m_drive_system.TankDrive(m_left_motorspeed, m_right_motorspeed);

  /*
  std::cout << "button pressed " << std::endl;
  std::cout << m_joystick.GetRawButton(11) << std::endl;
  std::cout << m_joystick.GetRawButtonPressed(12) << std::endl;
  */
 
  /*if(m_xbox.GetBButton()){
    // For every pixel
    for (int i = 0; i < kLength; i++) {
    //   // Calculate the hue - hue is easier for rainbows because the colour
    //   // shape is a circle so only one value needs to precess
      const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
    //   // Set the value
      m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    // // Increase by to make the rainbow "move"
    firstPixelHue += 3;
    // // Check bounds
    firstPixelHue %= 180;

     // Set the LEDs
    m_led.SetData(m_ledBuffer);
  }

  //purple
  if(m_xbox.GetXButton()){
    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetHSV(320, 255, 60);
    }
     // Set the LEDs
    m_led.SetData(m_ledBuffer);
  }*/

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m_encoder_left.Reset();
  m_encoder_right.Reset();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
        
 if(m_encoder_left.GetDistance() <= AUTO_BACK_DIST || m_encoder_right.GetDistance() <= AUTO_BACK_DIST)
      {
        m_drive_system.TankDrive(0,0);

      }
      else{
        m_drive_system.TankDrive(-0.5,05); // previous .8
      }             

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void BalancingMode() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
