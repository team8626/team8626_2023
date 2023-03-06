// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;


public class CloseClawCommand extends CommandBase {
  private ClawSubsystem m_claw;
  private PIDController m_pidController;
  private int m_targetAngle;
  private Timer m_timer = new Timer();
  private static boolean m_forced;

  public CloseClawCommand(ClawSubsystem claw, boolean forced) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    m_claw = claw;
    m_forced = forced;

    m_pidController = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);
    m_targetAngle = ClawConstants.kHardCloseAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  m_timer.start();  

    // Claw subsystem isn't homed, open until it's homed
    if(!m_claw.isHomed()){
      m_claw.homeSubsystem();
    }
    m_pidController.setSetpoint(m_targetAngle);
    m_pidController.setTolerance(5); // Tolerance 5 degrees
  
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Run the PID Controller
      // double pidOut = m_pidController.calculate(m_claw.getAngle());
      // m_claw.setMotor(pidOut);
      m_claw.setMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stop();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;
    if(m_timer.hasElapsed(3.0)){
      retval = true;
    }
    //return  m_pidController.atSetpoint();
    return retval;
  }
}

