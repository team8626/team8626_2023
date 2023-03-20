// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class DummyCommand extends CommandBase {
  private int count = 0;

  public DummyCommand(ClawSubsystem claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("---------- BEGIN DummyTest ---------");

  }

  @Override
  public void execute() {
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("---------- END DummyTest ---------");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(count > 100){
      return true;
    }
    return false;
  }
}

