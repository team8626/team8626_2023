// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ActiveClawSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private WPI_VictorSPX m_clawMotor = new WPI_VictorSPX(ClawConstants.kCANMotorClaw);   // Neverest
  private WPI_VictorSPX m_leftMotor; //= new WPI_VictorSPX(6);  // Left
  private WPI_VictorSPX m_rightMotor; // = new WPI_VictorSPX(11);  // Right
  // Declare Pneumatic Solenoid
  private final Solenoid m_clawCylinder = new Solenoid(PneumaticsModuleType.REVPH, 3);

  private boolean m_activeIntake = false;
  private boolean m_isClosed = true;
  
  // Class Constructor
  public ActiveClawSubsystem() {

    if(m_activeIntake){
      m_leftMotor = new WPI_VictorSPX(6);  // Left
      m_rightMotor = new WPI_VictorSPX(11);  // Right
      m_leftMotor.setInverted(false);
      m_rightMotor.setInverted(true);
    }
    this.close();
  }


  public void initDashboard(){
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
  }

  public void updateDashboard(){
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
  }

  public boolean m_isClosed() {
    return m_isClosed;
  }

  public void close(){ 
    m_clawMotor.set(1);
    m_clawCylinder.set(true);
  }

  public void open(){ 
    m_clawMotor.set(-1);
    m_clawCylinder.set(true);
  }

  public void stopMotor(){ 
    m_clawMotor.set(0);
  }

  public void closePneumatic(){
    m_clawCylinder.set(true);
    m_isClosed = true;
  }

  public void openPneumatic(){
    m_clawCylinder.set(false);
    m_isClosed = false;
  }

  public void startSpin() {
    if(m_activeIntake){
      m_leftMotor.set(0.4);
      m_rightMotor.set(0.4);
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

  public boolean isOpened() {
    return !m_isClosed();
  }

  public void setClosed() {
    m_isClosed = true;
  } 
  
  @Override
  public void periodic() {
  }
}