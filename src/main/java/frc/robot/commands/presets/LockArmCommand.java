// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.subsystems.CloseClawCommand;
import frc.robot.commands.subsystems.ExtendArmCommand;
import frc.robot.commands.subsystems.MoveElevatorTopCommand;
import frc.robot.commands.subsystems.RetractArmCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class LockArmCommand  extends SequentialCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ElevatorSubsystem m_elevator;
  ClawSubsystem m_claw;
  LEDManagerSubsystem m_ledManager;

public LockArmCommand (ElevatorSubsystem elevator, ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, LEDManagerSubsystem LEDManager) {
  m_elbow = elbow;
  m_extender = extender;
  m_elevator = elevator;
  m_ledManager = LEDManager;
  m_claw = claw;
  

  addCommands(
      new CloseClawCommand(m_claw),
      new RetractArmCommand(m_extender),
      new MoveElevatorTopCommand(m_elevator),
      new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kLockArmElbowAngle, true),
      new WaitCommand(3),
      new ExtendArmCommand(m_extender)
    );
  }
}
