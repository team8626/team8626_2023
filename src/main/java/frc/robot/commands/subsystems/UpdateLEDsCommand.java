// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDManagerSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html



public class UpdateLEDsCommand extends InstantCommand {
  private final LEDManagerSubsystem m_ledManager;
  private final byte m_color;
  
  public UpdateLEDsCommand(LEDManagerSubsystem ledManager, byte newColor) {
    m_ledManager = ledManager;
    m_color = newColor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ledManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledManager.setColor(m_color);
  }
}
