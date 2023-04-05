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
import frc.robot.Robot;
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

  private enum ClawStates {
    CLOSING, CLOSED,
    OPENING, OPENED
  }

  private ClawStates m_clawState = ClawStates.CLOSED; 

  private final PneumaticSubsystem m_pneumatic;

  private boolean m_isClosed = true;
  private Timer m_timer = new Timer();
  private Timer m_motorTimer = new Timer();
  private boolean m_pressureOK = false;
  private boolean m_motorActive = true;
  private boolean m_activeIntake = true;

  // Class Constructor
  public ClawSubsystem(PneumaticSubsystem pneumatic) {

    m_pneumatic = pneumatic;

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
    m_motorTimer.reset();
    this.close();
  }

  public void initDashboard(){
    SmartDashboard.putString("Claw State", getStateString());
    SmartDashboard.putBoolean("Claw Motor", m_motorActive);
  }

  public void updateDashboard(){
    SmartDashboard.putString("Claw State", getStateString());
    SmartDashboard.putBoolean("Claw Motor", m_motorActive);
  }

  public void close(){ 
    m_motorTimer.reset();
    m_motorTimer.start();

    m_timer.reset();
    m_timer.start();

    m_clawMotor.set(1);
        m_motorActive = true;

    m_clawCylinder.set(false);
    setClawState(ClawStates.CLOSING);
  }

  public void open(){ 
    m_motorTimer.reset();
    m_motorTimer.start();

    m_timer.reset();
    m_timer.start();

    m_clawMotor.set(-1);
    m_motorActive = true;

    m_clawCylinder.set(true);
    setClawState(ClawStates.OPENING);
  }

  private void stopMotor(){ 
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
    return (m_clawState == ClawStates.CLOSED);
  }

  public boolean isOpened() {
    return (m_clawState == ClawStates.OPENED);
  }

  public ClawStates getClawState() {
    return m_clawState;
    }

  public void setClawState(ClawStates state) {
      m_clawState = state;
    }

  public String getStateString() {
      switch(m_clawState) {
  
        default:
        return "None Found";
  
        case CLOSING:
        return "closeING";
        case CLOSED:
        return "closeED";
        case OPENING:
        return "openING";
        case OPENED:
        return "openED";
        
      }
  
    }

  @Override
  public void periodic() {
    if(m_motorTimer.hasElapsed(3.0)){
      this.stopMotor();
    }

    // Check current Pressure.
    if(m_pneumatic.getPressure() > PneumaticConstants.kMinClawPSI){
      m_pressureOK = true;
    } else {
      m_pressureOK = false;
    }

    // Pressure OK? Start a timer is not done yet.
    if(m_pressureOK && ((m_clawState == ClawStates.OPENING))||(m_clawState == ClawStates.CLOSING)){
      if (m_timer.get() == 0) {
        m_timer.reset();
        m_timer.start();
      }
    } 
    // Low Pressure? Stop timer
    else {
      m_timer.stop();
      m_timer.reset();
    }
        
    // Pressure is up, wait for the air to go through the cylinders
    if(m_pressureOK) {
      if(m_clawState == ClawStates.OPENING) {
        if(m_timer.hasElapsed(0.25)){
          setClawState(ClawStates.OPENED);
          m_timer.stop();
          m_timer.reset();
        }
      }

      else if (m_clawState == ClawStates.CLOSING) {
        if(m_timer.hasElapsed(.2)){
          setClawState(ClawStates.CLOSED);
          m_timer.stop();
          m_timer.reset();
       }
      }
    } 
  }
}