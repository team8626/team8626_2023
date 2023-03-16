// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.DriveSpeed;

public class DriveAdjustmentModeCommand extends CommandBase {
  private static SwerveDriveSubsystem m_drivetrain;
private static double m_speedFactor;
  /** Creates a new DriveAdjustmentModeCommand. */
  public DriveAdjustmentModeCommand(SwerveDriveSubsystem drivetrain, DriveSpeed speed) {
 m_drivetrain = drivetrain;
  switch(speed) {
  case LOWEST_SPEED:
  m_speedFactor = SwerveDriveConstants.kLowestSpeedFactor;
  break;
  case LOW_SPEED:
  m_speedFactor = SwerveDriveConstants.kLowSpeedFactor;
  break;

}


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_drivetrain.setPowerFactor(m_speedFactor);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setPowerFactor(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
