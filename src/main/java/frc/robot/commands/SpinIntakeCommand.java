// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ActiveClawSubsystem;


public class SpinIntakeCommand extends CommandBase {
  private ActiveClawSubsystem m_claw;

  public SpinIntakeCommand(ActiveClawSubsystem claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    m_claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_claw.startSpin();
    System.out.printf("[SpinIntakeCommand] Start Spin\n");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopSpin();
    System.out.printf("[SpinIntakeCommand] Stop Spin\n");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

