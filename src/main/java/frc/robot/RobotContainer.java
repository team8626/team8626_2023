// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.lang.ModuleLayer.Controller;

import org.xml.sax.SAXException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOControls;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.ElevatorTestCommand;
import frc.robot.commands.MoveElevatorBottomCommand;
import frc.robot.commands.MoveElevatorTopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KitbotDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drive Types
 */
enum DriveType {
  SWERVE,
  KITBOT
}
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private DriveSubsystem m_robotDrive = null;
  private static SubsystemBase m_robotDrive = null;
  private static final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  

  private final static DriveType m_driveType = DriveType.KITBOT; // SWERVE or KITBOT
  //private final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();
  //private final KitBotDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();

  // Define controllers
  // private final XboxController m_xBoxController = new XboxController(IOControls.kXboxControllerPort);
  private final PS4Controller m_xBoxController = new PS4Controller(IOControls.kXboxControllerPort);
  // private final Joystick m_flightJoystick = new Joystick(IOControls.kJoystickControllerPort);
  private final Joystick m_flightJoystick = new Joystick(1);

  // Autonomous Mode Selection
  // TODO: Add Autonomous dashboard and controls here
  
  private final static DashBoard m_dashboard = new DashBoard();
  private final static Autonomous m_autoControl = new Autonomous(m_dashboard, m_robotDrive);
 
  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Instantiate the drivetrain
    switch(m_driveType){
      case SWERVE /*kSwerve*/: 
        m_robotDrive = new SwerveDriveSubsystem();
        break;

      case KITBOT /*kKitBot*/: 
        m_robotDrive = new KitbotDriveSubsystem();
        break;
    }
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
    // Swerve Drive Train Specific Bindings
    //
    if(m_robotDrive instanceof SwerveDriveSubsystem){
      // Pressing Right Bumper set Swerve Modules to Cross (X) Position
      new JoystickButton(m_xBoxController, Button.kR1.value)
      .whileTrue(new RunCommand(
          () -> ((SwerveDriveSubsystem)m_robotDrive).setX(),
          m_robotDrive));
    }

    //
    // KitBot Drive Train Specific Bindings
    //
    if(m_robotDrive instanceof KitbotDriveSubsystem){

    }
    (new Trigger(() -> m_flightJoystick.getTriggerPressed()))
    .toggleOnTrue(new ElevatorTestCommand(m_elevator));

    Trigger button11 = new JoystickButton(m_flightJoystick, 11);
    button11.toggleOnTrue(new MoveElevatorBottomCommand(m_elevator));

    Trigger button12 = new JoystickButton(m_flightJoystick, 12);
    button12.toggleOnTrue(new MoveElevatorTopCommand(m_elevator));



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

    //
    // Swerve Drive Train Specific Bindings
    //
    // if(m_robotDrive.isSwerve()){
    if(m_robotDrive instanceof SwerveDriveSubsystem){
      m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> ((SwerveDriveSubsystem)m_robotDrive).drive(
              MathUtil.applyDeadband(-m_xBoxController.getLeftY(), 0.06),
              MathUtil.applyDeadband(-m_xBoxController.getLeftX(), 0.06),
              MathUtil.applyDeadband(-m_xBoxController.getRightX(), 0.06),
              true),
          m_robotDrive));
    }

    //
    // KitBot Drive Train Specific Bindings
    //
    if(m_robotDrive instanceof KitbotDriveSubsystem){
      m_robotDrive.setDefaultCommand(
      // Joystick controls the robot.
      // Speed is controlled by Y axis, Rotation is controlled by Y Axis, 
      new RunCommand(
        () -> ((KitbotDriveSubsystem)m_robotDrive).drive(
              MathUtil.applyDeadband(-m_flightJoystick.getY(), 0.06),
              MathUtil.applyDeadband(-m_flightJoystick.getX(), 0.06)),
          m_robotDrive));
    }
    m_elevator.setDefaultCommand(new ElevatorMoveCommand(() ->  m_flightJoystick.getY(), m_elevator));
  }


  /**
   * Get Start command from the autonomous controller (Dashboard)
   * TODO: Insert Last year's code here
   */
  public Command getAutonomousCommand() {
    Command retval = null;
     try {
       retval = m_autoControl.getStartCommand();
     } catch (IOException e) {
       // TODO Auto-generated catch block
       e.printStackTrace();
     }
    return retval;
  }
}
