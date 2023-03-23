// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticConstants;

public class ArmExtensionSubsystem extends SubsystemBase {
  // Pneumatics
  private final Solenoid m_armCylinder = new Solenoid(PneumaticConstants.kPCMtype, ArmConstants.kSolenoidChannel);

  // Subsystem State
  private boolean m_extended;

  public ArmExtensionSubsystem() {

     // Initialize Start-Up State
     this.retract();
     m_extended = false;
  }

  // Initialize Dashboard
  public void initDashboard(){
    if(m_extended){
      SmartDashboard.putString("Arm Length", "EXTENDED");
    } else {
      SmartDashboard.putString("Arm Length", "RETRACTED");
    }
  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
    if(m_extended){
      SmartDashboard.putString("Arm Length", "EXTENDED");
    } else {
      SmartDashboard.putString("Arm Length", "RETRACTED");
    }
  } 

  public boolean isExtended() {
    return m_extended;
  }
  
  // Extend the arm extension
  public void extend(){
    // Extend Cylinders
    m_armCylinder.set(true);

    // Update Subsytem Status
    m_extended = true;
  }
  
  // Retract the arm extension
  public void retract(){
    // Retract Cylinders
    m_armCylinder.set(false);
 
  // Update Subsytem Status
      m_extended = false;
  }

  @Override
  public void periodic() {
  }
}

