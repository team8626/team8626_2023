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
import frc.robot.subsystems.ElevatorSubsystem;

public class SetStowPositionCommand extends SequentialCommandGroup {

ArmElbowSubsystem m_elbow;
ArmExtensionSubsystem m_extender;
ClawSubsystem m_claw;
ElevatorSubsystem m_elevator;

  public SetStowPositionCommand(ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, ElevatorSubsystem elevator) {
    m_elbow = elbow;
    m_extender = extender;
    m_claw = claw;
    m_elevator = elevator;

  addCommands(
      new CloseClawCommand(m_claw), 
      new ParallelCommandGroup(new SetArmElbowCommand(m_elbow, ArmConstants.kStowedElbowAngle), new RetractArmCommand(m_extender), new MoveElevatorTopCommand(m_elevator))
             );

  }
  
}
