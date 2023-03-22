// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ExtendArmCommand;
import frc.robot.commands.MoveElevatorTopCommand;
import frc.robot.commands.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class ReadyForGrid2  extends SequentialCommandGroup {
  ArmElbowSubsystem m_elbow;
  ArmExtensionSubsystem m_extender;
  ClawSubsystem m_claw;
  ElevatorSubsystem m_elevator;
  LEDManagerSubsystem m_ledManager;
  
  public ReadyForGrid2 (ElevatorSubsystem elevator,ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw,  LEDManagerSubsystem LEDManager) {
    m_elevator = elevator;
    m_elbow = elbow;
    m_extender = extender;
    m_claw = claw;
    m_ledManager = LEDManager;

    addCommands(
        new ParallelCommandGroup( new MoveElevatorTopCommand(m_elevator),
                                  new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kMiddleGridElbowAngle)),
        new ExtendArmCommand(m_extender)
      );
  }
}
