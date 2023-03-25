// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDManagerConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  DriverStation.Alliance m_allianceColor =  DriverStation.getAlliance();

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
    SmartDashboard.putNumber("Balance kP", 0.005);
    SmartDashboard.putNumber("Balance kI", 0);
    SmartDashboard.putNumber("Balance kD", 0.00001);
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
    m_robotContainer.m_ledManager.setAllianceColor();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** 
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class. 
   */
  @Override
  public void autonomousInit() {
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
  public void autonomousPeriodic() {}

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
    m_robotContainer.m_drive.setBalancekP(SmartDashboard.getNumber("Balance kP", 0.005));
    m_robotContainer.m_drive.setBalancekI(SmartDashboard.getNumber("Balance kI", 0));
    m_robotContainer.m_drive.setBalancekD(SmartDashboard.getNumber("Balance kD", 0.00001));
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
