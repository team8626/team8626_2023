// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.presets.SetStowPositionCommand;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewAutoBalance extends SequentialCommandGroup {
  
  public NewAutoBalance(SwerveDriveSubsystem swerve, ElevatorSubsystem elevator, LEDManagerSubsystem LEDs, ArmElbowSubsystem elbow, ArmExtensionSubsystem extender, ClawSubsystem claw) {

    addCommands(
      new SetStowPositionCommand(elevator, elbow, extender, claw, LEDs).withTimeout(1),
      new RunCommand(() -> swerve.drive(-0.2, 0, 0, true, false), swerve).withTimeout(3),
      new BalanceTest(swerve, LEDs).withTimeout(8),
      new InstantCommand(() -> swerve.setReverseStart(!(swerve.getReverseStart()))
      )
    );
  }
}
