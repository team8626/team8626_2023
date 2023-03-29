// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class RetractArmCommand extends CommandBase {
  
  private final ArmExtensionSubsystem m_extender;


  public RetractArmCommand(ArmExtensionSubsystem extender) {
    m_extender = extender;
    addRequirements(extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_extender.retract();
  }

  @Override
  public boolean isFinished() {
    return m_extender.isRetracted();
  }


  }

