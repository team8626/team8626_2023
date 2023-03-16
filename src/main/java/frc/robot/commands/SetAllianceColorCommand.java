// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDManagerSubsystem;

public class SetAllianceColorCommand extends InstantCommand {
  private static LEDManagerSubsystem m_LEDs;
  public SetAllianceColorCommand(LEDManagerSubsystem LEDs) {
    m_LEDs = LEDs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDs.setAllianceColor();
  }
}