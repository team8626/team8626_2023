// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import org.xml.sax.SAXException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOControlsConstants;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.commands.BottomGridSetupCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DoubleSubstationPickupCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.MiddleGridSetupCommand;
import frc.robot.commands.MoveElevatorBottomCommand;
import frc.robot.commands.MoveElevatorTopCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.SetArmElbowCommand;
import frc.robot.commands.SetFloorPositionCommand;
import frc.robot.commands.SetStowPositionCommand;
import frc.robot.commands.SetTraversePositionCommand;
import frc.robot.commands.TopGridSetupCommand;
import frc.robot.commands.UpdateLEDsCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.KitbotDriveSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
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
  public final ArmExtensionSubsystem m_extender = new ArmExtensionSubsystem();
  public final LEDManagerSubsystem m_ledManager = new LEDManagerSubsystem();
  // private final ArmExtensionSubsystem m_armExtension = new ArmExtensionSubsystem();
  
  private Alliance m_allianceColor;


  private final static DriveType m_driveType = DriveType.SWERVE; // SWERVE or KITBOT
  //private final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();
  //private final KitBotDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();

  // Define controllers
  // private final XboxController m_xBoxController = new XboxController(IOControls.kXboxControllerPort);
  private final XboxController m_xBoxController = new XboxController(IOControlsConstants.kXboxControllerPort);
  private final Joystick m_flightJoystick = new Joystick(IOControlsConstants.kJoystickControllerPort);
  private final Joystick m_buttonBox = new Joystick(IOControlsConstants.kButtonBoxPort);

  // Declare Events Map
  private static HashMap<String, Command> eventMap = new HashMap<>();

  // Autonomous Mode Selection  
  private static DashBoard m_dashboard;
  private static Autonomous m_autoControl;
 
  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    configureEventsMaps();

    // Instatiate the Dashbpard
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
   
    m_allianceColor = DriverStation.getAlliance();
    m_autoControl = new Autonomous(m_dashboard, m_robotDrive);

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
      new JoystickButton(m_xBoxController, Button.kRightBumper.value)
      .whileTrue(new RunCommand(
          () -> ((SwerveDriveSubsystem)m_robotDrive).setX(),
          m_robotDrive));
    }

    //
    // KitBot Drive Train Specific Bindings
    //
    if(m_robotDrive instanceof KitbotDriveSubsystem){

    }

  


// TODO: Button commands are not updated
    Trigger topLeftButton = new JoystickButton(m_buttonBox, 1);
    topLeftButton.toggleOnTrue(new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator));

    Trigger middleLeftButton = new JoystickButton(m_buttonBox, 2);
    middleLeftButton.toggleOnTrue(new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator));

    Trigger bottomLeftButton = new JoystickButton(m_buttonBox, 3);
    bottomLeftButton.toggleOnTrue(new BottomGridSetupCommand(m_elbow, m_extender, m_elevator));

    Trigger topCenterButton = new JoystickButton(m_buttonBox, 4);
    topCenterButton.toggleOnTrue(new DoubleSubstationPickupCommand(m_elbow, m_extender, m_elevator));

    Trigger middleCenterButton = new JoystickButton(m_buttonBox, 5);
    middleCenterButton.toggleOnTrue(new SetFloorPositionCommand(m_elbow, m_extender, m_claw, m_elevator));

    Trigger bottomCenterButton = new JoystickButton(m_buttonBox, 6);
    bottomCenterButton.toggleOnTrue(new SetTraversePositionCommand(m_elbow, m_extender, m_claw, m_elevator));

    // LED Control Buttons
    new JoystickButton(m_buttonBox, 7) 
    .onTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE));
    // .onTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED));

    new JoystickButton(m_buttonBox, 8) 
      .onTrue(m_allianceColor == DriverStation.Alliance.Blue? 
              new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
              new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED)
            );
    // .onTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED));
    new JoystickButton(m_buttonBox, 9) 
    .onTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCUBE));
    // .onTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE));

    Trigger button4 = new JoystickButton(m_flightJoystick, 4);
    button4.toggleOnTrue(new OpenClawCommand(m_claw));
    
    Trigger button5 = new JoystickButton(m_flightJoystick, 5);
    button5.toggleOnTrue(new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator));

    Trigger button6 = new JoystickButton(m_flightJoystick, 6);
    button6.toggleOnTrue(new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator));


  

    Trigger button10 = new JoystickButton(m_flightJoystick, 10);
    button10.toggleOnTrue(new DoubleSubstationPickupCommand(m_elbow, m_extender, m_elevator));

    Trigger button11 = new JoystickButton(m_flightJoystick, 11);
     button11.toggleOnTrue(new MoveElevatorBottomCommand(m_elevator));

     Trigger button12 = new JoystickButton(m_flightJoystick, 12);
    button12.toggleOnTrue(new MoveElevatorTopCommand(m_elevator));

  

    // Toggle Auto-Balancing mode ON/OFF
    // TODO: Ned's code goes here

  }

  /**
   * Set Default Commands for Subsystems if needed...
   */
  private void configureDefaultCommands() {}

  /**
   * Populate Autonomous Event map...
   */
  private void configureEventsMaps() {
      // Populate Autonomous Event map
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("marker2", new PrintCommand("Passed marker 2"));
      eventMap.put("marker3", new PrintCommand("Passed marker 3"));
      eventMap.put("marker4", new PrintCommand("Passed marker 4"));
      eventMap.put("marker5", new PrintCommand("Passed marker 5"));
  }
  

      
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

    // Load trajectoryt file "Example Path.path" and generate it with a max velocity of 1 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("path_1", new PathConstraints(1.0, 3.0));

    retval = new FollowPathWithEvents(
      ((SwerveDriveSubsystem)m_robotDrive).followTrajectoryCommand(trajectory, true),
      trajectory.getMarkers(),
      eventMap);
      
    return retval;
  }
}
