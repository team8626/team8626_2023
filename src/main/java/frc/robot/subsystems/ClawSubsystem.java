// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private WPI_VictorSPX m_clawMotor = new WPI_VictorSPX(ClawConstants.kCANMotorClaw);
  // Declare Pneumatic Solenoid
  //private final Solenoid m_clawCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 15, 8);
  private final Solenoid m_clawCylinder = new Solenoid(PneumaticsModuleType.REVPH, 3);

  private boolean m_isClosed = true;
  
  // Class Constructor
  public ClawSubsystem() {
    m_clawMotor.setNeutralMode(NeutralMode.Coast); // Using Coast since we have a pneumatic cylinder
    // m_cylinderOpen.set(true);
    // m_cylinderClose.set(false);
    this.close();
  }


  public void initDashboard(){
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
  }

  public void updateDashboard(){
    SmartDashboard.putBoolean("Claw Closed", !m_isClosed);
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

  public void setClosed() {
    m_isClosed = true;
  } 

  public boolean m_isClosed() {
    return m_isClosed;
  }

  public boolean isOpened() {
    return !m_isClosed();
  }

  @Override
  public void periodic() {
  }
}