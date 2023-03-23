// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PneumaticConstants;

public class ClawSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private WPI_VictorSPX m_clawMotor = new WPI_VictorSPX(ClawConstants.kCANMotorClaw);
  private WPI_VictorSPX m_leftMotor; //= new WPI_VictorSPX(6);  // Left
  private WPI_VictorSPX m_rightMotor; // = new WPI_VictorSPX(11);  // Right
  
  // Declare Pneumatic Solenoid
  //private final Solenoid m_clawCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 15, 8);
  private final Solenoid m_clawCylinder = new Solenoid(PneumaticConstants.kPCMtype, ClawConstants.kSolenoidChannel);

  private boolean m_isClosed = true;
  private Timer m_timer = new Timer();
  private boolean m_motorActive = true;
  private boolean m_activeIntake = true;

  // Class Constructor
  public ClawSubsystem() {
    m_clawMotor.setNeutralMode(NeutralMode.Coast); // Using Coast since we have a pneumatic cylinder
    // m_cylinderOpen.set(true);
    // m_cylinderClose.set(false);
    if(m_activeIntake){
      m_leftMotor = new WPI_VictorSPX(6);  // Left
      m_rightMotor = new WPI_VictorSPX(11);  // Right
      m_leftMotor.setInverted(false);
      m_rightMotor.setInverted(true);
    }

    m_timer.reset();
    this.close();
  }


  public void initDashboard(){
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
    SmartDashboard.putBoolean("Claw Motor", m_motorActive);
  }

  public void updateDashboard(){
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
    SmartDashboard.putBoolean("Claw Motor", m_motorActive);
  }

  public void close(){ 
    m_timer.reset();
    m_timer.start();

    m_clawMotor.set(1);
        m_motorActive = true;

    m_clawCylinder.set(false);
  }

  public void open(){ 
    m_timer.reset();
    m_timer.start();

    m_clawMotor.set(-1);
    m_motorActive = true;

    m_clawCylinder.set(true);
  }

  private void stopMotor(){ 
    m_timer.stop();
    m_timer.reset();
    
    m_clawMotor.set(0);
    m_motorActive = false;

  }

  public void startSpin() {
    if(m_activeIntake){
      m_leftMotor.set(0.8);
      m_rightMotor.set(0.8);
    }
  }

  public void stopSpin() {
    if(m_activeIntake){
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }

  public void reverseSpin() {
    if(m_activeIntake){
      m_leftMotor.set(-1);
      m_rightMotor.set(-1);
    }
  }

  public boolean isClosed() {
    return m_isClosed;
  }

  public boolean isOpened() {
    return !m_isClosed;
  }

  @Override
  public void periodic() {
    if(m_timer.hasElapsed(3.0)){
      this.stopMotor();
    }
  }
}