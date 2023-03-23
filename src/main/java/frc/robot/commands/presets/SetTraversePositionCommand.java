// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.subsystems.MoveElevatorBottomCommand;
import frc.robot.commands.subsystems.RetractArmCommand;
import frc.robot.commands.subsystems.SetArmElbowCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;

public class SetTraversePositionCommand extends SequentialCommandGroup {

ArmElbowSubsystem m_elbow;
ArmExtensionSubsystem m_extender;
ClawSubsystem m_claw;
ElevatorSubsystem m_elevator;
LEDManagerSubsystem m_ledManager;

  public SetTraversePositionCommand(ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw, ElevatorSubsystem elevator, LEDManagerSubsystem LEDManager) {
    m_elbow = elbow;
    m_extender = extender;
    m_claw = claw;
    m_elevator = elevator;
    m_ledManager = LEDManager;

  addCommands(
    // Moves until waitcommand ends then it will move on to elbow and extension and instantly restart elevator
    new ParallelRaceGroup(new WaitCommand(1), new MoveElevatorBottomCommand(m_elevator)), 

    new ParallelCommandGroup(
    new SetArmElbowCommand(m_elbow, m_ledManager, ArmConstants.kTraverseElbowAngle), 
    new RetractArmCommand(m_extender), 
    new MoveElevatorBottomCommand(elevator)
    )
    
    );

  }

}
