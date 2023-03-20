// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;

// 
public class BalanceLockCommand extends SequentialCommandGroup {

  SwerveDriveSubsystem m_drivetrain;

  public BalanceLockCommand(SwerveDriveSubsystem drivetrain) {
    m_drivetrain = drivetrain;
    addCommands(new BalanceTest(m_drivetrain), new BalanceLockCommand(m_drivetrain));
  }
}
