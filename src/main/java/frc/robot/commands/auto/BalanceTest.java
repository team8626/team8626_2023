// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceTest extends PIDCommand {
  private PIDController m_PID;
  private LEDManagerSubsystem m_ledManager;
  private SwerveDriveSubsystem m_drivetrain;
  private Timer m_timer;

  /** Creates a new BalanceCommandRemastered. */
  public BalanceTest(SwerveDriveSubsystem drivetrain, LEDManagerSubsystem ledManager) {
    super(
        // The controller that the command will use
        // Greater Boston Values: 0.0055 P 0.0007 D
        new PIDController(0.0055, 0, 0.0007),
        // This should return the measurement
        () -> drivetrain.getPitch(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drivetrain.drive(-output, 0, 0, false, true);
        });
        m_drivetrain = drivetrain;
        m_ledManager = ledManager;
        m_timer = new Timer();
        addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("---------- BEGIN BalanceTest ---------");
    m_ledManager.setColor(LEDManagerConstants.kColorWHITE);
    m_timer.reset();
    m_timer.start();
    m_PID = getController();
    m_PID.setTolerance(SwerveDriveConstants.kBalancedPositionTolerance /* , SwerveDriveConstants.kBalancedVelocityTolerance */);
  }

  @Override
  public void end(boolean interrupted) {
    m_ledManager.setAllianceColor();
    System.out.println("---------- END BalanceTest ---------");
  }

  @Override
  public void execute() {
    if(m_PID.atSetpoint() && (m_timer.hasElapsed(1))){
        m_drivetrain.setX();
    } else {
      m_timer.reset();
      m_timer.start();
    }
  }

  @Override
  public boolean isFinished() {
    // Never Terminate.
    // In Autonomous, this is the last command.
    // In Teleoperated, it should be on a Trigger().toggleOnTrue()
    return false;
  }
}
