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
#include <frc/Joystick.h>
#include <frc/Encoder.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Timer.h>
#include <frc/AddressableLED.h>

//LED includes

  static constexpr int kLength = 5;

  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{4};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;

#define BOOST_SPEED (0.7)
#define NORMAL_SPEED (0.55)
#define AUTO_BACK_DIST (200)
#define VERTICAL_ENCODER_PULSE (0.05) //check
#define HORIZONTAL_ENCPODER_PULSE (0.05) //check
#define CLAWSPEED (0.5) //check
#define VERTICAL_STOP_POSITION (1038) //check
#define HORIZONTAL_STOP_POSITION (701) //check
#define CLAW_OPEN_TIME_SECONDS (10)

void BalancingMode();

// Config vars
// bool m_is2022Robot = true;
bool m_balancing_mode = false;

bool m_cone_deposit = false;
units::time::second_t timer_time;
frc::Timer m_claw_timer = frc::Timer(); 

// Motors
frc::PWMSparkMax m_Spark_left{0};
frc::PWMSparkMax m_Spark_right{8};

frc::PWMSparkMax m_vertical{1}; //double check port no and controller
frc::PWMSparkMax m_horizontal{2};
frc::PWMSparkMax m_claw{3};

// Encoders
frc::Encoder m_encoder_left{8,9, true};
frc::Encoder m_encoder_right{6,7, false};
frc::Encoder m_encoder_vertical{0,1};
frc::Encoder m_encoder_horizontal{2,3};

// Gyro
frc::ADXRS450_Gyro m_gyro{frc::SPI::Port::kMXP};

// Drive Config
frc::DifferentialDrive m_drive_system = frc::DifferentialDrive{m_Spark_left,m_Spark_right};

// Xbox Controller (should be mapped to port 1)
frc::XboxController m_xbox{1};

//Joy Stick Controller (should be mapped to port 0)
frc::Joystick m_joystick{0};

// Cameras
cs::UsbCamera m_camera_claw;
cs::UsbCamera m_camera_drive;

// Network tables
nt::NetworkTableEntry m_cameraSelection;


void Robot::RobotInit() {

  // Default to a length of 60, start empty output
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
  
  if(m_gyro.IsConnected() == true){
    m_gyro.Calibrate();
  }
  else{
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "********** Gyro not connected ***********" << std::endl;
    std::cout << std::endl << std::endl << std::endl;
  }

  m_camera_claw = frc::CameraServer::StartAutomaticCapture("Claw Camera", 0); //check ports
  m_camera_drive = frc::CameraServer::StartAutomaticCapture("Drive Camera", 1);
  m_camera_claw.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  m_camera_drive.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  m_cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("CameraSelection");
  m_cameraSelection.SetString(m_camera_claw.GetName());

  //Check port mapping for joystick and gamepad
  if (m_xbox.GetType() != frc::GenericHID::kXInputGamepad)
  {
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "********** xbox controller mapped to wrong port ***********" << std::endl;
    std::cout << std::endl << std::endl << std::endl;

    while(true);
    // FIXME - instead of infinite while loop, should throw an exception
  }
  if (m_joystick.GetType() != frc::GenericHID::kHIDJoystick)
  {
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "*********** joystick mapped to wrong port ***********" << std::endl;
    std::cout << std::endl << std::endl << std::endl;

    while(true);
    // FIXME - instead of infinite while loop, should throw an exception
  };

 // BalancingMode{} FIXME - Work out how to give this instructions pls :(
  
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
  // Mode debug
  //m_autoSelected = m_chooser.GetSelected();
  //m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  // fmt::print("Auto selected: {}\n", m_autoSelected);

  // Init encoders
  m_encoder_left.Reset();
  m_encoder_right.Reset();

  m_encoder_left.SetDistancePerPulse(0.0521); // distance per pulse in inches
  m_encoder_right.SetDistancePerPulse(0.0521); // distance per pulse in inches

  m_encoder_vertical.Reset();
  m_encoder_vertical.SetDistancePerPulse(VERTICAL_ENCODER_PULSE);
  m_encoder_horizontal.Reset();
  m_encoder_horizontal.SetDistancePerPulse(HORIZONTAL_ENCPODER_PULSE);

  m_claw_timer.Reset();   //FIXME - check if reset needs to be called before or after start()
  m_claw_timer.Start();

}

void Robot::AutonomousPeriodic() {
  // TODO: place game piece
  if(m_cone_deposit == false)
  {
  //extend vertical thingy (check what distance cone needs move up)
  //extend horizontal thingy (check distance)
  //release tha claaaw & set m_cone_deposit == true
  //retract everything???
    if(m_encoder_vertical.GetDistance() < VERTICAL_STOP_POSITION)
    {
      m_vertical.Set(0.5);
      timer_time = units::time::second_t(m_claw_timer.Get()) + units::time::second_t(CLAW_OPEN_TIME_SECONDS);
    }
    else{
      m_vertical.Set(0);
    }

    if(m_encoder_horizontal.GetDistance() < HORIZONTAL_STOP_POSITION)
    {
      m_horizontal.Set(0.5);
    }
    else{
      m_horizontal.Set(0);
    }

    if( (m_encoder_horizontal.GetDistance() >= HORIZONTAL_STOP_POSITION) && 
        (m_encoder_vertical.GetDistance()   >= VERTICAL_STOP_POSITION  ) )
    {
      m_claw.Set(-CLAWSPEED);
      if(m_claw_timer.HasElapsed(timer_time))
      {
        m_claw.Set(0);
        m_cone_deposit = true;
      }
    }

  }
  
  if(m_cone_deposit == true)
  {
    // IF BALANCING
    if (m_balancing_mode == true)
    {
      BalancingMode();
    }
      
    // Drive toward charging station while less than set distance
    /*if(m_encoder_left.GetDistance() >= AUTO_BACK_DIST && m_encoder_right.GetDistance() >= AUTO_BACK_DIST)
    {
      // // Check gyro angle greater than 9 degrees
      if(m_gyro.GetAngle() > 9)
      {
      // If gyro angle is  then start balancing == set a flag
      m_balancing_mode = true; 
      m_drive_system.TankDrive(0.7,-0.7);
      }
      else 
      {
        // If not keep driving
        m_drive_system.TankDrive(0.5,-0.5);
      }
    else{
      // Stop driving
      m_drive_system.TankDrive(0, 0);
    }*/
  }
}

void Robot::TeleopInit() {
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

  //Verticle Extension

  if (m_joystick.GetPOV()==0)
  {
    m_vertical.Set(0.6);

  }

  if (m_joystick.GetPOV()==180)
  {
    m_vertical.Set(-0.6);

    if (m_encoder_vertical.GetDistance()<=0)
    {
      m_vertical.Set(0);
    }
    
  }
  
  //HORIZONTAL EXTENSION
  if (m_joystick.GetRawAxis(1)==1)
  {
    m_horizontal.Set(0.6);
  }

  if (m_joystick.GetRawAxis(1)==-1)
  {
    
    if (m_encoder_horizontal.GetDistance()<=0)
    {
      m_horizontal.Set(0);
    }
    else{
      m_horizontal.Set(-0.6);

    }
    
  }

  //CLAW GRABBY THINGY

  if (m_joystick.GetTrigger() == 1)
  {
    m_claw.Set(CLAWSPEED);
  }

  if (m_joystick.GetTop() == 1)
  {
    m_claw.Set(-CLAWSPEED);
  }

  //Cameras
  if (m_xbox.GetXButton()) {
    m_cameraSelection.SetString(m_camera_claw.GetName());
  }
  if (m_xbox.GetBButton()) {
    m_cameraSelection.SetString(m_camera_drive.GetName());
  }

  /*
  std::cout << "button pressed " << std::endl;
  std::cout << m_joystick.GetRawButton(11) << std::endl;
  std::cout << m_joystick.GetRawButtonPressed(12) << std::endl;
  */


  if(m_joystick.GetRawButton(11)){
    // For every pixel
    for (int i = 0; i < kLength; i++) {
    //   // Calculate the hue - hue is easier for rainbows because the color
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
  if(m_joystick.GetRawButton(12)){
    for (int i = 0; i < kLength; i++) {
      m_ledBuffer[i].SetHSV(320, 255, 60);
    }
     // Set the LEDs
    m_led.SetData(m_ledBuffer);
  }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void BalancingMode() {

    double robotPitchAngle = m_gyro.GetAngle();
    // if gyro is positive drive forward at slow speed (30%) until angle is zero
    if(robotPitchAngle > 0)
    {
      m_drive_system.TankDrive(-0.4, 0.4);
    }
    // if gyro is negative drive backwards at slow speed
    else if (robotPitchAngle < 0)
    {
      m_drive_system.TankDrive(-0.4, 0.4);
    }
    // if gyro is level STOP MOTORS
    else
    {
      m_drive_system.TankDrive(0, 0);
    }
  
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
