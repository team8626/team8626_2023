// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.utils.CyberUtils;
import frc.robot.Constants.LEDManagerConstants;


public class BruteForceBalanceCommand extends CommandBase {
  private SwerveDriveSubsystem m_drive;
  private LEDManagerSubsystem m_ledManager;
  private Timer m_timer = new Timer();
  private double m_toleranceDegrees = 3.0;
  private boolean m_wentOver = false;
  private double m_previousPitchSign = -1;
  private double m_startSpeed = 0.15;
  private double m_finalApproachSpeed = 0.05;
  private double m_driveSpeed;


  public BruteForceBalanceCommand(SwerveDriveSubsystem drive, LEDManagerSubsystem ledManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_ledManager = ledManager;
    addRequirements(drive, ledManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_previousPitchSign = Math.signum(m_drive.getPitch());
    m_wentOver = false;
    m_driveSpeed = m_startSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ledManager.setColor(LEDManagerConstants.kColorWHITE);

    if(m_previousPitchSign != Math.signum(m_drive.getPitch())) {
      m_wentOver = true;
    }
    // We are not there yet...
    if(CyberUtils.deadband(m_drive.getPitch(), m_toleranceDegrees) != 0){
      // We are getting close
      if((Math.abs(m_drive.getPitch()) - m_toleranceDegrees) < (2*m_toleranceDegrees) // Less than 3x tolerance
        || m_wentOver)
      {
        m_driveSpeed = m_finalApproachSpeed;
      } 
      // Not close yet...
      else {
        m_driveSpeed = m_startSpeed;
      }
      m_drive.drive(m_driveSpeed * Math.signum(m_drive.getPitch()), 0, 0, false, true);
    }
    // This is it!
    else {
      m_drive.drive(0, 0, 0, false, true);
      m_drive.setX();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledManager.setAllianceColor();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    boolean neverFinish = true;
    if(!neverFinish && (CyberUtils.deadband(m_drive.getPitch(), m_toleranceDegrees) == 0)){
      retval = true;
    }
    return retval;
  }
}

