// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {

// Pneumatics
  // TODO Should eventually move to a "Constants" class
  // TODO Same values from the intake last year
  private final DoubleSolenoid m_Cylinder = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8 /* 0 */, 9 /* 1 */);

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

  // Set assembly passive
  public void rest(){
    m_Cylinder.set(Value.kOff);
  }

  public boolean isExtended() {
    return m_extended;
  }
  
  // Extend the arm extension
  public void extend(){
    // Extend Cylinders
    m_Cylinder.set(Value.kForward);

    // Update Subsytem Status
    m_extended = true;
      System.out.println("[Arm] Extended");
  }
  
  // Retract the arm extension
  public void retract(){
    // Retract Cylinders
    m_Cylinder.set(Value.kReverse);
 
  // Update Subsytem Status
      m_extended = false;
      System.out.println("[Arm] Retracted");
  }

  @Override
  public void periodic() {
  }
}

