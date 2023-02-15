// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveCommand extends CommandBase {
  ElevatorSubsystem m_elevator;
  private final DoubleSupplier m_speed;
 

  public ElevatorMoveCommand( DoubleSupplier speed, ElevatorSubsystem elevator) {
    m_elevator = elevator;
    m_speed = speed;
    addRequirements(m_elevator);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_elevator.setMotor(m_speed.getAsDouble());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotor(0);
  }
}