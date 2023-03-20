// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOControlsConstants;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.BalanceTest;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.DriveAdjustmentModeCommand;
import frc.robot.commands.MiddleGridSetupCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.SetArmElbowCommand;
import frc.robot.commands.SetFloorPositionCommand;
import frc.robot.commands.SetStowPositionCommand;
import frc.robot.commands.TopGridSetupCommand;
import frc.robot.commands.UpdateLEDsCommand;
import frc.robot.commands.auto.ReadyForGrid2;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem.ItemType;
import frc.robot.subsystems.SwerveDriveSubsystem.DriveSpeed;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ClawSubsystem m_claw = new ClawSubsystem();
  public final ArmElbowSubsystem m_elbow = new ArmElbowSubsystem();
  public final ArmExtensionSubsystem m_extender = new ArmExtensionSubsystem();
  public final LEDManagerSubsystem m_ledManager = new LEDManagerSubsystem();
  // private final ArmExtensionSubsystem m_armExtension = new ArmExtensionSubsystem();
  
  private Alliance m_allianceColor;

  // Define controllers
  private final XboxController m_xboxController = new XboxController(IOControlsConstants.kXboxControllerPort);
  private final Joystick m_flightJoystick = new Joystick(IOControlsConstants.kJoystickControllerPort);
  private final Joystick m_buttonBox = new Joystick(IOControlsConstants.kButtonBoxPort);

  // Declare Events Map
  public HashMap<String, Command> eventMap = new HashMap<>();

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
    

    // Predefined Arm positions for Game Pieces delivery
    // new JoystickButton(m_buttonBox, 1) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager)));
    Trigger topLeftButton= new JoystickButton(m_buttonBox, 1);
    topLeftButton.toggleOnTrue(new TopGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // new JoystickButton(m_buttonBox, 2) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager)));
    Trigger middleLeftButton = new JoystickButton(m_buttonBox, 2);
    middleLeftButton.toggleOnTrue(new MiddleGridSetupCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // new JoystickButton(m_buttonBox, 3) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new BottomGridSetupCommand(m_elbow, m_extender, m_elevator, m_ledManager)));
    //Trigger bottomLeftButton = new JoystickButton(m_buttonBox, 3);
   // bottomLeftButton.toggleOnTrue(new BottomGridSetupCommand(m_elbow, m_extender, m_elevator, m_ledManager));

    // Predefined Arm positions for Game Pieces pickup
   // Trigger topCenterButton = new JoystickButton(m_buttonBox, 4);
   // topCenterButton.toggleOnTrue(new DoubleSubstationPickupCommand(m_elbow, m_extender, m_elevator, m_ledManager));

    //Trigger middleCenterButton = new JoystickButton(m_buttonBox, 5);
   // middleCenterButton.toggleOnTrue(new SetFloorPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // Safe position for traveling
  //  Trigger bottomCenterButton = new JoystickButton(m_buttonBox, 6);
  //  bottomCenterButton.toggleOnTrue(new SetTraversePositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

    // LED Control Buttons
    // new JoystickButton(m_buttonBox, 7) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE), 
    //                                 new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CONE)));
    Trigger topRightButton = new JoystickButton(m_buttonBox, 7);
    topRightButton.toggleOnTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED));
                                
    // new JoystickButton(m_buttonBox, 8) 
    //   .onTrue(m_allianceColor == DriverStation.Alliance.Blue? 
    //           new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
    //           new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED)
    //         );
    /* */
    Trigger middleRightButton = new JoystickButton(m_buttonBox, 8);
    middleRightButton.toggleOnTrue(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorRAINBOW));

    // new JoystickButton(m_buttonBox, 9) 
    // .onTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCUBE), 
    //                                  new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CUBE)));
    Trigger bottomRightButton = new JoystickButton(m_buttonBox, 9);
    bottomRightButton.toggleOnTrue(new ParallelCommandGroup(new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCUBE), 
                                     new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CUBE)));

  // Claw Controls
  Trigger xboxLeftBumper = new JoystickButton(m_xboxController, XboxControllerConstants.kLeftBumper);
  xboxLeftBumper.onTrue(new ParallelCommandGroup(m_allianceColor == DriverStation.Alliance.Blue? 
                                                    new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
                                                    new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED),
                                                new OpenClawCommand(m_claw)));

  Trigger xboxRightBumper = new JoystickButton(m_xboxController, XboxControllerConstants.kRightBumper);
  xboxRightBumper.onTrue(new CloseClawCommand(m_claw));
  
  /*  Manual start of Balancing Command
  Trigger xboxBButton = new JoystickButton(m_xboxController, XboxControllerConstants.kBButton);
  xboxBButton.toggleOnTrue(new BalanceCommandRemastered(m_robotDrive, m_ledManager));
*/
  // Pressing X Button  set Swerve Modules to Cross (X) Position
  Trigger xButton = new JoystickButton(m_xboxController, XboxControllerConstants.kXButton);
  xButton.toggleOnTrue(new ParallelCommandGroup( 

  new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCONE),

  new RunCommand(
        () -> ((SwerveDriveSubsystem)m_robotDrive).setX(),
          m_robotDrive))

          );
  
  
  // Stow the Arm
  Trigger xboxStartButton = new JoystickButton(m_xboxController, XboxControllerConstants.kStartButton);
  xboxStartButton.toggleOnTrue(new BalanceTest(m_robotDrive));
  

  Trigger xboxBButton = new JoystickButton(m_xboxController, XboxControllerConstants.kBButton);
  xboxBButton.toggleOnTrue(new DriveAdjustmentModeCommand(m_robotDrive, DriveSpeed.LOW_SPEED));
   
  Trigger xboxAButton = new JoystickButton(m_xboxController, XboxControllerConstants.kAButton);
  xboxAButton.toggleOnTrue(new DriveAdjustmentModeCommand(m_robotDrive, DriveSpeed.LOWEST_SPEED));

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
      eventMap.put("ReadyForGrid2", new ReadyForGrid2(m_elevator, m_elbow, m_extender, m_claw, m_ledManager));
      eventMap.put("SetupForIntake", new SetFloorPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));
      eventMap.put("StowArm", new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager));

  }

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
        () -> (m_robotDrive).drive(
            MathUtil.applyDeadband(-m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
            MathUtil.applyDeadband(-m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
            MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
            false,
            true),
        m_robotDrive));
  
  }

  /**
   * Get Start command from the autonomous controller (Dashboard)
   */
  public Command getAutonomousCommand() {
    // This is default behavior is nothing works... DO NOT ERASE
    // Command retval = new DeliverFromGrid(m_elevator, m_elbow, m_extender, m_claw, m_ledManager);

    //PathPlannerTrajectory trajectory = PathPlanner.loadPath("Start9_Cone4_Balance", new PathConstraints(2.0, 2.0));

    // THAT WORKS, DOES ALL STEPS, MOVED TO DASHBOARD/AUTONOMOUS
    // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start9_Cone4_Balance", new PathConstraints(4, 3));
    // Command retval = new SequentialCommandGroup(
    //                             // Starting the game. Make sure the claw is closed and get ready for delivery
    //                             new CloseClawCommand(m_claw),
    //                             new ReadyForGrid2(m_elevator, m_elbow, m_extender, m_claw, m_ledManager),
    //                             new WaitCommand(.25),
    //                             new OpenClawCommand(m_claw),

    //                             // Go to pickup next piece
    //                             new FollowPathWithEvents( m_robotDrive.followTrajectoryCommand( pathGroup.get(0), true),
    //                                                   pathGroup.get(0).getMarkers(),
    //                                                   eventMap),

    //                             new CloseClawCommand(m_claw),
    //                             new FollowPathWithEvents( m_robotDrive.followTrajectoryCommand( pathGroup.get(0), true),
    //                             pathGroup.get(1).getMarkers(),
    //                             eventMap),

    //                            // At this point, we should be ready to deliver
    //                             new WaitCommand(.25),
    //                             new OpenClawCommand(m_claw),

    //                             // Go to Charging Station
    //                             new FollowPathWithEvents( m_robotDrive.followTrajectoryCommand( pathGroup.get(0), true),
    //                             pathGroup.get(2).getMarkers(),
    //                             eventMap),
    //                             new BalanceTest(m_robotDrive));

    // THAT WORKS, SHOWS MIDDLE MARKERS
    // Command retval = new FollowPathWithEvents(m_robotDrive.followTrajectoryCommand( trajectory, true),
    //                                         trajectory.getMarkers(),
    //                                         eventMap);

          
      // DriverStation.reportError("[AUTO] Enable to load trajectory", e.getStackTrace());

      // CODE BELLOW WORKS UNTIL END OF 2ND TRAJECTORY
      /* 
      new SequentialCommandGroup(new AutoStartPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager), 
                                 new FollowPathWithEvents(
                                    m_robotDrive.followTrajectoryCommand(trajectory1, true),
                                    trajectory1.getMarkers(),
                                    eventMap),
                                  new OpenClawCommand(m_claw),
                                  
                                  // new ParallelCommandGroup(
                                    new SetStowPositionCommand(m_elbow, m_extender, m_claw, m_elevator, m_ledManager),
                                    new FollowPathWithEvents(
                                      m_robotDrive.followTrajectoryCommand(trajectory2, false),
                                      trajectory2.getMarkers(),
                                      eventMap)
                                    // )
                                    ,

                                  new FollowPathWithEvents(
                                    m_robotDrive.followTrajectoryCommand(trajectory3, false),
                                    trajectory3.getMarkers(),
                                    eventMap));


 */
    Command startCommand = new InstantCommand();
    
    try{
      startCommand = m_autoControl.getStartCommand(this);
    }
    catch(IOException e){
      DriverStation.reportError("Could not open auto-trajectory file", e.getStackTrace());
      startCommand = new SequentialCommandGroup(
                                    // Starting the game. Make sure the claw is closed and get ready for delivery
                                    new CloseClawCommand(m_claw),
                                    new ReadyForGrid2(m_elevator, m_elbow, m_extender, m_claw, m_ledManager),
                                    new WaitCommand(.25),
                                    new OpenClawCommand(m_claw));
    }
    return startCommand;
  }
}

