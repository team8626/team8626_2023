// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceCommand extends CommandBase {
   Timer m_delayTimer = new Timer();

  // Move to constants when not lazy
  private static final double balanceThresholdDegrees = 2.5;
  private static final double pitchRateMinimumDegrees = 1;

  private SwerveDriveSubsystem m_drivetrain;
  private LEDManagerSubsystem m_ledManager;

  private static boolean m_balanceMode;
  private static boolean m_abort;
  private static boolean m_isAborted;
  private static double m_xAxisRate;
  private static double m_currPitchRate;

  public BalanceCommand(SwerveDriveSubsystem drivetrain, LEDManagerSubsystem ledManager, boolean abort) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_ledManager = ledManager;
    m_abort = abort;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledManager.setColor(LEDManagerConstants.kColorWHITE);
    m_balanceMode = true;
        
    // Starts with a fake pitch rate above the threshold so it can run
    // without instantly being shutdown
    m_currPitchRate = pitchRateMinimumDegrees + 0.1;
    if(m_abort) 
      m_delayTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    double robotPitch = m_drivetrain.getPitch();
        
    // if not actively balancing
    if ( !m_balanceMode && 
    (Math.abs(robotPitch) >= 
    Math.abs(balanceThresholdDegrees))) {
      m_balanceMode = true;
    }
    // if actively balancing
    else if (m_balanceMode && 
        (Math.abs(robotPitch) <= 
          Math.abs(balanceThresholdDegrees))) {
      m_balanceMode = false;
    }
    // math stuff for tank drive values
    if(m_balanceMode) {
      double pitchAngleRadians = robotPitch * (Math.PI / 180.0);
      m_xAxisRate = (Math.sin(pitchAngleRadians) * -1);
    }
    // If you want to abort if it isn't balancing (its gonna fly off)
    if (m_abort){
      // Only update the rate with a real value if it started driving for a bit
      if(m_delayTimer.get() > 2) m_currPitchRate = (m_drivetrain).getPitchRate();
      // Checking if it is balancing
      if(m_currPitchRate > pitchRateMinimumDegrees){
      m_drivetrain.drive(m_xAxisRate, 0,0,false, true);
      }
      // If its stagnant its gonna drive off so abort
      else {
        m_isAborted = true;
      }
    }
    // If you don't want the robot to abort
    else {
      m_drivetrain.drive(m_xAxisRate, 0,0,false, true);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_isAborted) {
      System.out.println("Balancing aborted");
    }
    else{
      System.out.println("Balancing stopped"); 
    }
    m_delayTimer.stop();

    // Set LEDS to Alliance Color
    m_ledManager.setAllianceColor(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isAborted;
  }
}
