// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmElbowSubsystem;

public class ArmElbowTestCommand extends CommandBase {
  Timer m_Timer;
  ArmElbowSubsystem a = new ArmElbowSubsystem();
  /** Creates a new ArmElbowTestCommand. */
  public ArmElbowTestCommand() {
    m_Timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(m_Timer.get() < 4){
      a.setMotor(0.3);
   } else if(m_Timer.get() < 9 && m_Timer.get() >= 5){
    a.setMotor(0.6);
   } else if(m_Timer.get() < 15 && m_Timer.get() >= 10){
    a.setMotor(0.9);
   }
     // while()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 18;
  }
}
