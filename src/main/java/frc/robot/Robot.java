// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDManagerConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  DriverStation.Alliance m_allianceColor = DriverStation.getAlliance();
  private Timer m_gameTimer = new Timer();

  public Command getAutonomousCommand() {
    return m_robotContainer.getAutonomousCommand();
  }

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.
    // This will perform all our button bindings,
    // and put ourmautonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    // Start the Camera Server
    // try {
    //   UsbCamera camera = CameraServer.startAutomaticCapture();
    //   camera.setResolution(640, 480);
    // } catch (VideoException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** 
   * This function is called once each time the robot enters Disabled mode. 
   */
  @Override
  public void disabledInit() {
    m_robotContainer.m_ledManager.setColor(LEDManagerConstants.kColorPINK);
  }

  @Override
  public void disabledPeriodic() {
    if(DriverStation.isDSAttached()){
      m_robotContainer.m_ledManager.setAllianceColor();
    } else {
      m_robotContainer.m_ledManager.setColor(LEDManagerConstants.kColorPINK);
    }
  }

  @Override
  public void disabledExit() {}

  /** 
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class. 
   */
  @Override
  public void autonomousInit() {
    m_gameTimer.start();
    m_robotContainer.m_ledManager.setAllianceColor();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  }
 
  /** 
   * This function is called periodically during autonomous. 
   */
  @Override
  public void autonomousPeriodic() {
    if(m_gameTimer.get() > 14.5) m_robotContainer.m_drive.setX();
   
  }

  /** 
   * This function is called when leaving autonomous mode. 
   */
  @Override
  public void autonomousExit() {
  }

  /** 
   * This function is called when operator control is started. 
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.configureTeleopDefaultCommands();
    
    // Set LEDS to AllIance Color
    m_robotContainer.m_ledManager.setAllianceColor();
  }

  /** 
   * This function is called periodically during operator control. 
   */
  @Override
  public void teleopPeriodic() {}

  /** 
   * This function is called when leaving operator control mode. 
   */
  @Override
  public void teleopExit() {
    m_robotContainer.m_ledManager.setColor(LEDManagerConstants.kColorPINK); 
    m_gameTimer.stop();
  }

  /**
   * This function is called when test mode is started.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** 
   * This function is called periodically during test mode. 
   */
  @Override
  public void testPeriodic() {}

  /** 
   * This function is called when leaving test mode. 
   */
  @Override
  public void testExit() {}
}
