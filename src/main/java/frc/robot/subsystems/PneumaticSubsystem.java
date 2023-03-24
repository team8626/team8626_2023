// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class PneumaticSubsystem extends SubsystemBase {
  // Pneumatics
  private final PneumaticHub m_hub = new PneumaticHub();

  public PneumaticSubsystem() {
    m_hub.clearStickyFaults();
    m_hub.enableCompressorAnalog(PneumaticConstants.kMinPressure, PneumaticConstants.kMaxPressure);
  }

  // Initialize Dashboard
  public void initDashboard(){
    SmartDashboard.putNumber("Pneumatic Pressure", m_hub.getPressure(0));
  }

  // Update Dashboard(Called Periodically)
  public void updateDashboard(){
    SmartDashboard.putNumber("Pneumatic Pressure", m_hub.getPressure(0));
  }
}

