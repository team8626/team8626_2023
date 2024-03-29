// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.subsystems.ExtendArmCommand;
import frc.robot.commands.subsystems.MoveElevatorTopCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class TopGridSetupCommand  extends SequentialCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ElevatorSubsystem m_elevator;
  LEDManagerSubsystem m_ledManager;

public TopGridSetupCommand (ElevatorSubsystem elevator, ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, LEDManagerSubsystem LEDManager) {
  m_elbow = elbow;
  m_extender = extender;
  m_elevator = elevator;
  m_ledManager = LEDManager;
  

  addCommands(
      new MoveElevatorTopCommand(m_elevator),
      new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kTopGridElbowAngle, true),
      new WaitCommand(1),
      new ExtendArmCommand(m_extender)
    );
  }
}
