// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOControlsConstants;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.commands.auto.BalanceTest;
import frc.robot.commands.auto.BruteForceBalanceCommand;
import frc.robot.commands.presets.MiddleGridSetupCommand;
import frc.robot.commands.presets.BottomGridSetupCommand;
import frc.robot.commands.presets.DoubleSubstationPickupCommand;
import frc.robot.commands.presets.SetFloorPositionCommand;
import frc.robot.commands.presets.SetStowPositionCommand;
import frc.robot.commands.presets.SetTraversePositionCommand;
import frc.robot.commands.presets.TopGridSetupCommand;
import frc.robot.commands.subsystems.CloseClawCommand;
import frc.robot.commands.subsystems.DriveAdjustmentModeCommand;
import frc.robot.commands.subsystems.OpenClawCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.commands.subsystems.UpdateLEDsCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem.ItemType;
import frc.robot.subsystems.SwerveDriveSubsystem.DriveSpeed;
import frc.utils.CommandButtonController;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final SwerveDriveSubsystem m_drive = new SwerveDriveSubsystem();
  public final PneumaticSubsystem m_pneumatic = new PneumaticSubsystem();
  public final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  public final ClawSubsystem m_claw = new ClawSubsystem();
  public final ArmElbowSubsystem m_elbow = new ArmElbowSubsystem();
  public final ArmExtensionSubsystem m_extender = new ArmExtensionSubsystem();
  public final LEDManagerSubsystem m_ledManager = new LEDManagerSubsystem();

  // private final ArmExtensionSubsystem m_armExtension = new ArmExtensionSubsystem();
  
  private Alliance m_allianceColor;
  private boolean m_reverseStart = false;

  // Define controllers
  private final CommandXboxController m_xboxController = new CommandXboxController(IOControlsConstants.kXboxControllerPort);
  private final CommandButtonController m_buttonBox = new CommandButtonController(IOControlsConstants.kButtonBoxPort);
  // private final Joystick m_flightJoystick = new Joystick(IOControlsConstants.kJoystickControllerPort);

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
    m_autoControl = new Autonomous(m_dashboard, m_drive);

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
    
    // Claw Controls
    m_xboxController.leftBumper().onTrue(new OpenClawCommand(m_claw, m_elbow));
    m_xboxController.rightBumper().onTrue(new CloseClawCommand(m_claw));
  
    // Set Swerve Modules to Cross (X) Position
    m_xboxController.x().toggleOnTrue(new ParallelCommandGroup( 
          new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorPINK),
          new RunCommand(
                () -> ((SwerveDriveSubsystem)m_drive).setX(), m_drive))
    );
    
    // Start Balancing
    m_xboxController.start().toggleOnTrue(new SequentialCommandGroup( 
          new BalanceTest(m_drive, m_ledManager, true))
    );
    
    m_xboxController.back().onTrue(new InstantCommand(this::toggleControls));

    // Speed Adjustment
    m_xboxController.b().toggleOnTrue(new DriveAdjustmentModeCommand(m_drive, DriveSpeed.LOW_SPEED));
    m_xboxController.a().toggleOnTrue(new DriveAdjustmentModeCommand(m_drive, DriveSpeed.LOWEST_SPEED));

    // Predefined Arm positions for Game Pieces delivery
    m_buttonBox.button_1().onTrue(new TopGridSetupCommand(m_elevator, m_elbow, m_extender, m_ledManager));
    m_buttonBox.button_2().onTrue(new MiddleGridSetupCommand(m_elevator, m_elbow, m_extender, m_ledManager));
    m_buttonBox.button_3().onTrue(new BottomGridSetupCommand(m_elevator, m_elbow, m_extender, m_ledManager));

    m_buttonBox.button_4().onTrue(new DoubleSubstationPickupCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager));
    m_buttonBox.button_5().onTrue(new SetFloorPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager));
    m_buttonBox.button_6().onTrue(new SetTraversePositionCommand(m_elevator, m_elbow, m_extender, m_ledManager));

    m_buttonBox.button_7().onTrue(new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CONE));
    m_buttonBox.button_8().onTrue(m_allianceColor == DriverStation.Alliance.Blue? 
                                          new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCEBLUE):
                                          new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorALLIANCERED));
    m_buttonBox.button_9().onTrue(new SetArmElbowCommand(m_elbow, m_ledManager, ItemType.CUBE));
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
      eventMap.put("MiddleGridSetupCommand", new MiddleGridSetupCommand(m_elevator, m_elbow, m_extender, m_ledManager));
      eventMap.put("SetupForIntake", new SetFloorPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager));
      eventMap.put("StowArm", new SetStowPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager));
  }


  public void setReverseStart(){
    m_reverseStart = true;
  }
  
  /**
   * Set Default Commands for Subsystems 
   * THis is called when robot enters in teleop mode.
   * THis prevents controllers interfere with autonomous mode.
   */
  public void configureTeleopDefaultCommands(){

    // Force position at beginning of Teleop.
    Command startCommand = new SetStowPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager);
    startCommand.schedule();

    // TODO: THis hsould be handled by resetOdometry... Seems to be not working
    // This is a workaround...
    if(this.m_reverseStart){
      m_drive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> (m_drive).drive(
              MathUtil.applyDeadband(m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
              true,
              true),
          m_drive));
    } else {
      m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> (m_drive).drive(
                MathUtil.applyDeadband(-m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
                true,
                true),
            m_drive));     
    }
  }

  private void toggleControls(){
    m_reverseStart = !m_reverseStart;

    if(this.m_reverseStart){
      System.out.println("Reversing Controls");
      m_drive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> (m_drive).drive(
              MathUtil.applyDeadband(m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
              MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
              true,
              true),
          m_drive));
    } else {
      System.out.println("Stop Reversing Controls");
      m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> (m_drive).drive(
                MathUtil.applyDeadband(-m_xboxController.getLeftY(), IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getLeftX(), IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getRightX(), IOControlsConstants.kDriveDeadband),
                true,
                true),
            m_drive));     
    }
  }

  /**
   * Get Start command from the autonomous controller (Dashboard)
   */
  public Command getAutonomousCommand() {
    Command startCommand = new InstantCommand();
    
    try{
      startCommand = m_autoControl.getStartCommand(this);
    }
    catch(IOException e){
      DriverStation.reportError("Could not open auto-trajectory file", e.getStackTrace());
      startCommand = new SequentialCommandGroup(
                                    new CloseClawCommand(m_claw),
                                    new SetStowPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager),
                                    new MiddleGridSetupCommand(m_elevator, m_elbow, m_extender, m_ledManager),
                                    new WaitCommand(.25),
                                    new OpenClawCommand(m_claw, m_elbow),
                                    new SetStowPositionCommand(m_elevator, m_elbow, m_extender, m_claw, m_ledManager)
      );
    }
    return startCommand;
  }
}

