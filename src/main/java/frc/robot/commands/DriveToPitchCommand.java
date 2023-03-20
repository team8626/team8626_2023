// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.LEDManagerConstants;


public class DriveToPitchCommand extends CommandBase {
  private SwerveDriveSubsystem m_drive;
  private LEDManagerSubsystem m_ledManager;
  private float m_targetAngle;
  private Timer m_timer = new Timer();

  public DriveToPitchCommand(float targetAngle, SwerveDriveSubsystem drive, LEDManagerSubsystem ledManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_ledManager = ledManager;
    m_targetAngle = targetAngle;
    addRequirements(drive, ledManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledManager.setColor(LEDManagerConstants.kColorPINK);
    m_drive.drive(0.4, 0, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledManager.setAllianceColor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    if(m_drive.getPitch() >= m_targetAngle){
      retval = true;
    }

    return retval;
  }
}

