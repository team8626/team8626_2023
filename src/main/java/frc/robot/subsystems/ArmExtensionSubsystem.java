// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticConstants;

public class ArmExtensionSubsystem extends SubsystemBase {

  private enum ExtensionStates {
    RETRACTING, RETRACTED, 
    EXTENDING, EXTENDED,
    PASSIVE
  }

  private ExtensionStates m_extensionState = ExtensionStates.PASSIVE;
  private Timer m_timer = new Timer();
  private boolean m_pressureOK = false;
  // Pneumatics
  private final Solenoid m_armCylinder = new Solenoid(PneumaticConstants.kPCMtype, ArmConstants.kSolenoidChannel);
  private final PneumaticSubsystem m_pneumatic;


  public ArmExtensionSubsystem(PneumaticSubsystem pneumatic) {
    m_pneumatic = pneumatic;

     // Initialize Start-Up State
     m_timer.reset();
     this.retract();
  
  }

  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putString("Arm Length", getStateString());
    SmartDashboard.putBoolean("Pressure OK", m_pressureOK);

  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
      SmartDashboard.putString("Arm Length", getStateString());
      SmartDashboard.putBoolean("Pressure OK", m_pressureOK);
  } 
  
  public ExtensionStates getExtensionState() {
   return m_extensionState;
    }

  private void setExtensionState(ExtensionStates state) {
    m_extensionState = state;
  }

  public String getStateString() {
    switch(m_extensionState) {

      default:
      return "None Found";

      case RETRACTING:
      return "retractING";
      case RETRACTED:
      return "retractED";
      case EXTENDING:
      return "extendING";
      case EXTENDED:
      return "extendED";
      case PASSIVE:
      return "PASSIVE";

    }
  }

  // Extend the arm extension
  public void extend(){

    // Extend Cylinders
    m_armCylinder.set(true);

    m_timer.reset();
    m_timer.start();

    // Update Subsytem Status
    setExtensionState(ExtensionStates.EXTENDING);
  }
  
  // Retract the arm extension
  public void retract(){
    
    // Retract Cylinders
    m_armCylinder.set(false);
     
    m_timer.reset();
    m_timer.start();
  
    // Update Subsytem Status
    setExtensionState(ExtensionStates.RETRACTING);
  }

public boolean isExtended() {
  boolean retval = false;
  if(m_extensionState == ExtensionStates.EXTENDED){
    retval = true;
  }
  return retval;
}

public boolean isRetracted() {
  boolean retval = false;
  if(m_extensionState == ExtensionStates.RETRACTED){
    retval = true;
  }
  return retval;
}

  @Override
  public void periodic() {
    
    // Check current Pressure.
    if(m_pneumatic.getPressure() > PneumaticConstants.kMinArmPSI){
      m_pressureOK = true;
    } else {
      m_pressureOK = false;
    }

    // Pressure OK? Start a timer is not done yet.
    if(m_pressureOK && ((m_extensionState == ExtensionStates.EXTENDING))||(m_extensionState == ExtensionStates.RETRACTING)){
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
      if(m_extensionState == ExtensionStates.EXTENDING) {
        if(m_timer.hasElapsed(.4)){
          setExtensionState(ExtensionStates.EXTENDED);
          m_timer.stop();
          m_timer.reset();
        }
      }

      else if (m_extensionState == ExtensionStates.RETRACTING) {
        if(m_timer.hasElapsed(.4)){
          setExtensionState(ExtensionStates.RETRACTED);
          m_timer.stop();
          m_timer.reset();
       }
      }
    } 

    // if(Robot.isSimulation()){
    //   if(m_extensionState == ExtensionStates.EXTENDING) {
    //     setExtensionState(ExtensionStates.EXTENDED);
    //   }
    //   else if (m_extensionState == ExtensionStates.RETRACTING) {
    //     setExtensionState(ExtensionStates.RETRACTED);
    //   }
    // } 

}

  
    



  }


