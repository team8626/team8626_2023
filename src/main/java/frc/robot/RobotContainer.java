// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOControls;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();

  // Define controllers
  XboxController m_driverController = new XboxController(IOControls.kDriverControllerPort);

  // TODO: Ned
  //private final Joystick m_flightJoystick = new Joystick(Controller.kJoystickPort);
  //private final XboxController m_gameController = new XboxController(Controller.kGamepadPort); 
  
  // Autonomous Mode
  // TODO: Add Autonomous dashboard and controls here
  
  /** 
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */

  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // 
    new JoystickButton(m_driverController, Button.kR1.value)
    .whileTrue(new RunCommand(
        () -> m_robotDrive.setX(),
        m_robotDrive));


        // Toggle Auto-Balancing mode ON/OFF
    // TODO: Ned's code goes here

    // Grab (Close Claw)
    // TODO: Ned's code goes here

    // Release (Open Claw)
    // TODO: Ned's code goes here
  }

  /**
   * Set Default Commands for Subsystems if needed...
   */
  private void configureDefaultCommands() {}

  /**
   * Set Default Commands for Subsystems 
   * THis is called when robot enters in teleop mode.
   * THis prevents controllers interfere with autonomous mode.
   */
  public void configureTeleopDefaultCommands(){

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
              MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
              MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
              true),
          m_robotDrive));

    // TODO: Cleanup This? or Keep for Ensign Drive
    // m_drivetrain.setDefaultCommand(    
    // new ArcadeDriveCommand(
    //   () -> -m_flightJoystick.getX(), 
    //   () -> m_flightJoystick.getY(),
    //   // () -> m_gameController.getRightY(), 
    //   // () -> m_gameController.getRightX(),
    //   m_drivetrain));
  }


  /**
   * Get Start command from the autonomous controller (Dashboard)
   * TODO: Insert Last year's code here
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
