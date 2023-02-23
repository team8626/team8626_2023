// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmElbowSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class SetArmElbowCommand extends InstantCommand {
  private final ArmElbowSubsystem m_elbow;
  private final double m_angle;
  
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle) {
    m_elbow = elbow;
    m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_elbow.setAngle(m_angle);
  }


}
