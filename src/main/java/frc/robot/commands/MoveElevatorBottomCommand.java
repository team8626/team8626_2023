// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorBottomCommand extends CommandBase {
  /** Creates a new MoveElevatorTop. */
  ElevatorSubsystem m_elevator;
  public MoveElevatorBottomCommand(ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    m_elevator = elevator;
  }

  /*  Called when the command is initially scheduled.
  @Override
  public void initialize() {}
*/
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setMotor(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotor(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return  m_elevator.getBottomPressed();
  }
}
