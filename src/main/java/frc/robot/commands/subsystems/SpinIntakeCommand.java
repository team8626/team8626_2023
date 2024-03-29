// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;


public class SpinIntakeCommand extends CommandBase {
  private ClawSubsystem m_claw;

  public SpinIntakeCommand(ClawSubsystem claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    m_claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.startSpin();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

