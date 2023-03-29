// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticConstants;

public class ArmExtensionSubsystem extends SubsystemBase {

private enum ExtensionStates {
  RETRACTING, RETRACTED, 
  EXTENDING, EXTENDED,
  PASSIVE
}

  private ExtensionStates m_extensionState = ExtensionStates.PASSIVE;

  // Pneumatics
  private final Solenoid m_armCylinder = new Solenoid(PneumaticConstants.kPCMtype, ArmConstants.kSolenoidChannel);
  private final PneumaticSubsystem m_pneumatic;


  public ArmExtensionSubsystem(PneumaticSubsystem pneumatic) {
  m_pneumatic = pneumatic;


     // Initialize Start-Up State
     this.retract();
  
  }

  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putString("Arm Length", getStateString());
  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
      SmartDashboard.putString("Arm Length", getStateString());
   
  } 
  
  public ExtensionStates getExtensionState() {
   return m_extensionState;
    }

  public void setExtensionState(ExtensionStates state) {
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

    // Update Subsytem Status
    setExtensionState(ExtensionStates.EXTENDING);
  }
  
  // Retract the arm extension
  public void retract(){
    
    // Retract Cylinders
    m_armCylinder.set(false);
 
  // Update Subsytem Status
  setExtensionState(ExtensionStates.RETRACTING);
  }

public boolean isExtended() {
  return (m_extensionState == ExtensionStates.EXTENDED);
}

public boolean isRetracted() {
  return (m_extensionState == ExtensionStates.RETRACTED);
}

  @Override
  public void periodic() {

if(m_pneumatic.getPressure() > PneumaticConstants.kMinArmPSI) {

if(m_extensionState == ExtensionStates.EXTENDING) {
  Timer.delay(0.1);
  setExtensionState(ExtensionStates.EXTENDED);
}

else if (m_extensionState == ExtensionStates.RETRACTING) {
  Timer.delay(0.1);
  setExtensionState(ExtensionStates.RETRACTED);
}

} 

}

  
    



  }


