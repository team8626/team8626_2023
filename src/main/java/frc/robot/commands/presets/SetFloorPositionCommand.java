// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.subsystems.ExtendArmCommand;
import frc.robot.commands.subsystems.MoveElevatorBottomCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class SetFloorPositionCommand extends SequentialCommandGroup {

ArmElbowSubsystem m_elbow;
ArmExtensionSubsystem m_extender;
ClawSubsystem m_claw;
ElevatorSubsystem m_elevator;
LEDManagerSubsystem m_ledManager;

  public SetFloorPositionCommand(ElevatorSubsystem elevator, ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, LEDManagerSubsystem LEDManager) {
    m_elbow = elbow;
    m_extender = extender;
    m_claw = claw;
    m_elevator = elevator;
    m_ledManager = LEDManager;

  addCommands(
      new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kFloorElbowAngle),
      new MoveElevatorBottomCommand(elevator),
      new ExtendArmCommand(m_extender)

      // new OpenClawCommand(m_robot.m_claw, m_elbow),
    );
  }

}
