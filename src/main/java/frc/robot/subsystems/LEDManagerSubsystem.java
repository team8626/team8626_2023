// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDManagerConstants;

public class LEDManagerSubsystem extends SubsystemBase {
  private DigitalOutput m_pin1 = new DigitalOutput(LEDManagerConstants.kDIOCom1);
  private DigitalOutput m_pin2 = new DigitalOutput(LEDManagerConstants.kDIOCom2);
  private DigitalOutput m_pin3 = new DigitalOutput(LEDManagerConstants.kDIOCom3);

  private byte m_pin1Mask = 0b00000001;
  private byte m_pin2Mask = 0b00000010;
  private byte m_pin3Mask = 0b00000100;

  /** Class Constructor. */
  public LEDManagerSubsystem() {
  }

  public void setColor(byte newColor){
    m_pin1.set(((newColor & m_pin1Mask) == m_pin1Mask)? true : false);
    m_pin2.set(((newColor & m_pin2Mask) == m_pin2Mask)? true : false);
    m_pin3.set(((newColor & m_pin3Mask) == m_pin3Mask)? true : false);
  }

  public void setAllianceColor() {
      // Set LEDS to Alliance Color
      if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        setColor(LEDManagerConstants.kColorALLIANCEBLUE); 
        System.out.printf("[setAllianceColor] BLUE\n"); 

      } else {
        setColor(LEDManagerConstants.kColorALLIANCERED);
        System.out.printf("[setAllianceColor] RED\n"); 

      }
  }
  public byte getAllianceColor() {
    // Set LEDS to Alliance Color
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      return LEDManagerConstants.kColorALLIANCEBLUE;
    } else {
      return LEDManagerConstants.kColorALLIANCERED;
    }
}
}

