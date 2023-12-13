// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToCommand extends CommandBase {
  SwerveDriveSubsystem m_drive;
  LEDManagerSubsystem m_ledManager = new LEDManagerSubsystem();
  PIDController m_xPID = new PIDController(0, 0, 0);
  PIDController m_yPID = new PIDController(0, 0, 0);
  public DriveToCommand(SwerveDriveSubsystem drive, double x, double y, double kP, double kI, double kD) {
   m_drive = drive;
 
   m_xPID.setP(kP);
   m_xPID.setI(kI);
   m_xPID.setD(kD);

   m_yPID.setP(kP);
   m_yPID.setI(kI);
   m_yPID.setD(kD);

   m_xPID.setSetpoint(x);
   m_yPID.setSetpoint(y);

   m_xPID.setTolerance(0.1, 0.1);
   m_yPID.setTolerance(0.1, 0.1);

  }

  @Override
  public void initialize() {
  m_ledManager.setColor(LEDManagerConstants.kColorWHITE);
  }

  @Override
  public void execute() {
    m_drive.drive(m_xPID.calculate(m_drive.getPose().getX()), m_yPID.calculate(m_drive.getPose().getY()), 0, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    new StartEndCommand(() -> m_ledManager.setColor(LEDManagerConstants.kColorCONE), () -> m_ledManager.setAllianceColor(), m_ledManager).withTimeout(5).schedule();
  }

  @Override
  public boolean isFinished() {
    return m_xPID.atSetpoint() && m_xPID.atSetpoint();
  }
}
