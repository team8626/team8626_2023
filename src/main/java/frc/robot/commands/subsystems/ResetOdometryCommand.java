// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ResetOdometryCommand extends CommandBase {
  private SwerveDriveSubsystem m_drive;

  public ResetOdometryCommand(SwerveDriveSubsystem drive) {
    m_drive = drive;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_drive.getPose());
  }
}

