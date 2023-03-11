// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import javax.swing.plaf.TreeUI;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOControlsConstants;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.AutoStartPositionCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.BalanceCommandRemastered;
import frc.robot.commands.BottomGridSetupCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DoubleSubstationPickupCommand;
import frc.robot.commands.MiddleGridSetupCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.SetArmElbowCommand;
import frc.robot.commands.SetFloorPositionCommand;
import frc.robot.commands.SetStowPositionCommand;
import frc.robot.commands.SetTraversePositionCommand;
import frc.robot.commands.TopGridSetupCommand;
import frc.robot.commands.UpdateLEDsCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem.ItemType;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  // TODO: Should be null; fix initialization
  private static SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();
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
  // private final XboxController m_xboxController = new XboxController(IOControls.kXboxControllerPort);
  private final XboxController m_xboxController = new XboxController(IOControlsConstants.kXboxControllerPort);
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

    /* Instantiate the drivetrain
    switch(m_driveType){
      case SWERVE /* Swerve : 
        m_robotDrive = new SwerveDriveSubsystem();
        break;
 
     case KITBOT /* KitBot : 
        m_robotDrive = new KitbotDriveSubsystem();
        break;
    }
    */
   
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
    /* 
    if(m_robotDrive instanceof SwerveDriveSubsystem){
      // Pressing Right Bumper set Swerve Modules to Cross (X) Position
      new JoystickButton(m_xboxController, Button.kRightBumper.value)
      .whileTrue(new RunCommand(
          () -> ((SwerveDriveSubsystem)m_robotDrive).setX(),
          m_robotDrive));
    }
*/
    //
    // KitBot Drive Train Specific Bindings
    //
    /* 
    if(m_robotDrive instanceof KitbotDriveSubsystem){

    }
    */
    // Predefined Arm positions for Game Pieces delivery
    // new JoystickButton(m_buttonBox, 1) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager)));
    Trigger topLeftButton = new JoystickButton(m_buttonBox, 1);
    topLeftButton.toggleOnTrue(new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // new JoystickButton(m_buttonBox, 2) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager)));
    Trigger middleLeftButton = new JoystickButton(m_buttonBox, 2);
    middleLeftButton.toggleOnTrue(new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // new JoystickButton(m_buttonBox, 3) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new BottomGridSetupCommand(m_elbow, m_extender, m_elevator, m_ledManager)));
    Trigger bottomLeftButton = new JoystickButton(m_buttonBox, 3);
    bottomLeftButton.toggleOnTrue(new BottomGridSetupCommand(m_elbow, m_extender, m_elevator, m_ledManager));

    // Predefined Arm positions for Game Pieces pickup
    Trigger topCenterButton = new JoystickButton(m_buttonBox, 4);
    topCenterButton.toggleOnTrue(new DoubleSubstationPickupCommand(m_elbow, m_extender, m_elevator, m_ledManager));

    Trigger middleCenterButton = new JoystickButton(m_buttonBox, 5);
    middleCenterButton.toggleOnTrue(new SetFloorPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // Safe position for traveling
    Trigger bottomCenterButton = new JoystickButton(m_buttonBox, 6);
    bottomCenterButton.toggleOnTrue(new SetTraversePositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // LED Control Buttons
    // new JoystickButton(m_buttonBox, 7) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CONE)));
    Trigger topRightButton = new JoystickButton(m_buttonBox, 7);
    topRightButton.toggleOnTrue(new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CONE));
                                
    // new JoystickButton(m_buttonBox, 8) 
    //   .onTrue(m_allianceColor == DriverStation.Alliance.Blue? 
    //           new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
    //           new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED)
    //         );
    Trigger middleRightButton = new JoystickButton(m_buttonBox, 8);
    middleRightButton.toggleOnTrue(new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.NONE));

    // new JoystickButton(m_buttonBox, 9) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCUBE), 
    //                                  new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CUBE)));
    Trigger bottomRightButton = new JoystickButton(m_buttonBox, 9);
    bottomRightButton.toggleOnTrue(new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CUBE));

  // Claw Controls
  Trigger xboxLeftBumper = new JoystickButton(m_xboxController, XboxControllerConstants.kLeftBumper);
  xboxLeftBumper.onTrue(new ParallelCommandGroup(m_allianceColor == DriverStation.Alliance.Blue? 
                                                    new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
                                                    new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED),
                                                new OpenClawCommand(m_claw)));

  Trigger xboxRightBumper = new JoystickButton(m_xboxController, XboxControllerConstants.kRightBumper);
  xboxRightBumper.onTrue(new CloseClawCommand(m_claw));
  
  // Manual start of Balancing Command
  Trigger xboxBButton = new JoystickButton(m_xboxController, XboxControllerConstants.kBButton);
  xboxBButton.toggleOnTrue(new BalanceCommandRemastered(m_robotDrive, m_ledManager, false));

  // Stow the Arm
  Trigger xboxStartButton = new JoystickButton(m_xboxController, XboxControllerConstants.kStartButton);
  xboxStartButton.toggleOnTrue(new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

  /*   kLeftBumper(5),
    kRightBumper(6),
    kLeftStick(9),
    kRightStick(10),
    kA(1),
    kB(2),
    kX(3),
    kY(4),
    kBack(7),
    kStart(8);
*/

    /* 
    Trigger stickButton8 = new JoystickButton(m_flightJoystick, 8);
    stickButton8.toggleOnTrue(new ExtendArmCommand(m_extender));
    */

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
      eventMap.put("DeliverEvent", new SequentialCommandGroup(new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager), 
                                                                  new OpenClawCommand(m_claw), 
                                                                  new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager)));
      eventMap.put("StowEvent", new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));
      eventMap.put("BalanceEvent", new BalanceCommandRemastered(m_robotDrive, m_ledManager, false));
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
              MathUtil.applyDeadband(-m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
              true,
              true),
          m_robotDrive));
    }
  }

  /**
   * Get Start command from the autonomous controller (Dashboard)
   */
  public Command getAutonomousCommand() {
    Command retval = null;
    // try {
    //   retval =  
    //  new SequentialCommandGroup(new AutoStartPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager), 
    //  m_autoControl.getStartCommand((SwerveDriveSubsystem)m_robotDrive, eventMap));
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("trajectory1", new PathConstraints(1.0, 3.0));
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("trajectory2", new PathConstraints(1.0, 3.0));
    PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("trajectory3", new PathConstraints(1.0, 3.0));
      retval =  
      new SequentialCommandGroup(new AutoStartPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager), 
                                 new FollowPathWithEvents(
                                    m_robotDrive.followTrajectoryCommand(trajectory1, true),
                                    trajectory1.getMarkers(),
                                    eventMap),
                                  new OpenClawCommand(m_claw),
                                  
                                  new SequentialCommandGroup(
                                    new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager),
                                    new FollowPathWithEvents(
                                      m_robotDrive.followTrajectoryCommand(trajectory2, false),
                                      trajectory2.getMarkers(),
                                      eventMap)),
                                      
                                  new FollowPathWithEvents(
                                    m_robotDrive.followTrajectoryCommand(trajectory3, false),
                                    trajectory3.getMarkers(),
                                    eventMap));
    // } catch (IOException e) {
      // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    return retval;
  }

}

