// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class MiddleGridSetupCommand  extends ParallelCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ClawSubsystem m_claw;
  ElevatorSubsystem m_elevator;
  
public MiddleGridSetupCommand (ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, ElevatorSubsystem elevator) {
  m_elbow = elbow;
  m_extender = extender;
  m_claw = claw;
  m_elevator = elevator;

  
    addCommands(
        new RetractArmCommand(m_extender), new SetArmElbowCommand(m_elbow, ArmConstants.kMiddleGridElbowAngle, true), new MoveElevatorTopCommand(m_elevator)
        
               );

  }

}
