// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class OpenClawCommand extends CommandBase {
  private ClawSubsystem m_claw;
  private ArmElbowSubsystem m_elbow;
  private boolean invalidAngle;
  public OpenClawCommand(ClawSubsystem claw, ArmElbowSubsystem elbow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    m_claw = claw;
    m_elbow = elbow;
    invalidAngle = false;
    
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {
    invalidAngle = (m_elbow.getDesiredAngle() > ArmConstants.kMaxOpenClawAngle);
    m_claw.open();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_claw.isOpened() || invalidAngle;
  }
}
