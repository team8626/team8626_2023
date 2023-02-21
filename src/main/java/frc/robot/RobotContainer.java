// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.xml.sax.SAXException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOControlsConstants;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.MoveArmElbowCommand;
import frc.robot.commands.MoveElevatorBottomCommand;
import frc.robot.commands.MoveElevatorTopCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;
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
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ClawSubsystem m_claw = new ClawSubsystem();
  public final ArmElbowSubsystem m_elbow = new ArmElbowSubsystem();
  // private final ArmExtensionSubsystem m_armExtension = new ArmExtensionSubsystem();
  

  private final static DriveType m_driveType = DriveType.SWERVE; // SWERVE or KITBOT
  //private final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();
  //private final KitBotDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();

  // Define controllers
  // private final XboxController m_xBoxController = new XboxController(IOControls.kXboxControllerPort);
  private final PS4Controller m_xBoxController = new PS4Controller(IOControlsConstants.kXboxControllerPort);
  private final Joystick m_flightJoystick = new Joystick(IOControlsConstants.kJoystickControllerPort);

  // Autonomous Mode Selection  
  private static DashBoard m_dashboard;
  private static Autonomous m_autoControl = new Autonomous(m_dashboard, m_robotDrive);
 
  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
     m_dashboard = new DashBoard(this);

    // Instantiate the drivetrain
    switch(m_driveType){
      case SWERVE /* Swerve */: 
        m_robotDrive = new SwerveDriveSubsystem();
        break;
 
     case KITBOT /* KitBot */: 
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
    // if(m_robotDrive instanceof SwerveDriveSubsystem){
    //   // Pressing Right Bumper set Swerve Modules to Cross (X) Position
    //   new JoystickButton(m_xBoxController, Button.kR1.value)
    //   .whileTrue(new RunCommand(
    //       () -> ((SwerveDriveSubsystem)m_robotDrive).setX(),
    //       m_robotDrive));
    // }

    //
    // KitBot Drive Train Specific Bindings
    //
    if(m_robotDrive instanceof KitbotDriveSubsystem){

    }
    
    
    Trigger button5 = new JoystickButton(m_flightJoystick, 5);
    button5.toggleOnTrue(new OpenClawCommand(m_claw));

    Trigger button6 = new JoystickButton(m_flightJoystick, 6);
    button6.toggleOnTrue(new CloseClawCommand(m_claw));

    Trigger button7 = new JoystickButton(m_flightJoystick, 7);
    button7.toggleOnTrue(new MoveElevatorBottomCommand(m_elevator));

    Trigger button8 = new JoystickButton(m_flightJoystick, 8);
    button8.toggleOnTrue(new MoveElevatorTopCommand(m_elevator));

    Trigger button9 = new JoystickButton(m_flightJoystick, 9);
    button9.toggleOnTrue(new MoveArmElbowCommand(m_elbow, 90));

    Trigger button10 = new JoystickButton(m_flightJoystick, 10);
    button10.toggleOnTrue(new MoveArmElbowCommand(m_elbow, 180));

    // Trigger button11 = new JoystickButton(m_flightJoystick, 11);
    // button11.toggleOnTrue(new ExtendArmCommand(m_armExtension));

    // Trigger button12 = new JoystickButton(m_flightJoystick, 12);
    // button12.toggleOnTrue(new RetractArmCommand(m_armExtension));



    // Toggle Auto-Balancing mode ON/OFF
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
              MathUtil.applyDeadband(-m_xBoxController.getLeftY(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xBoxController.getLeftX(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xBoxController.getRightX(), IOControlsConstants.kDriveDeadband),
              false,
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
    // m_elevator.setDefaultCommand(new ElevatorMoveCommand(() ->  m_flightJoystick.getY(), m_elevator));
  }


  /**
   * Get Start command from the autonomous controller (Dashboard)
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
