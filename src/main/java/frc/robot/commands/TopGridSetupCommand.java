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
import frc.robot.subsystems.LEDManagerSubsystem;

public class TopGridSetupCommand  extends ParallelCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ClawSubsystem m_claw;
  ElevatorSubsystem m_elevator;
  LEDManagerSubsystem m_ledManager;

public TopGridSetupCommand (ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, ElevatorSubsystem elevator, LEDManagerSubsystem LEDManager) {
  m_elbow = elbow;
  m_extender = extender;
  m_claw = claw;
  m_elevator = elevator;
  m_ledManager = LEDManager;
  
    addCommands(
        new ExtendArmCommand(m_extender), new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kTopGridElbowAngle, true), new MoveElevatorTopCommand(m_elevator)
               );
  }

}
