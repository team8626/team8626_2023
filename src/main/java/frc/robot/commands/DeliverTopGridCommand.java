// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class DeliverTopGridCommand extends SequentialCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ClawSubsystem m_claw;
  
public DeliverTopGridCommand(ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw) {
  m_elbow = elbow;
  m_extender = extender;
  m_claw = claw;
    
    addCommands(
        new ParallelCommandGroup(new ExtendArmCommand(m_extender), new SetArmElbowCommand(m_elbow, ArmConstants.kTopGridElbowAngle)),
        new OpenClawCommand(m_claw)
               );
  }

}
