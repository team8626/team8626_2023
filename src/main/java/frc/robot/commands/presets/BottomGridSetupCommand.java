// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.subsystems.MoveElevatorTopCommand;
import frc.robot.commands.subsystems.RetractArmCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class BottomGridSetupCommand extends ParallelCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ElevatorSubsystem m_elevator;
  LEDManagerSubsystem m_ledManager;

  public BottomGridSetupCommand(ElevatorSubsystem elevator, ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, LEDManagerSubsystem LEDManager) {
    m_elbow = elbow;
    m_extender = extender;
    m_elevator = elevator;
    m_ledManager = LEDManager;

    addCommands(
      new RetractArmCommand(m_extender),
      new SequentialCommandGroup(
        new MoveElevatorTopCommand(m_elevator),
        new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kBottomGridElbowAngle, true)
        )
    );
  }
}
