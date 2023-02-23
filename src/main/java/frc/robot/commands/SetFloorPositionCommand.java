// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFloorPositionCommand extends ParallelCommandGroup {

ArmElbowSubsystem m_elbow;
ArmExtensionSubsystem m_extender;
ClawSubsystem m_claw;

  public SetFloorPositionCommand(ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw) {
    m_elbow = elbow;
    m_extender = extender;
    m_claw = claw;

  addCommands(
      new SetArmElbowCommand(m_elbow, ArmConstants.kFloorElbowAngle),
      new ExtendArmCommand(m_extender),
      new OpenClawCommand(m_claw)
             );
  }

}
