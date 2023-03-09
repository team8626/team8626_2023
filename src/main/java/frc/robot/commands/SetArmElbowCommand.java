// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem.ItemType;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class SetArmElbowCommand extends InstantCommand {
  private final ArmElbowSubsystem m_elbow;
  private double m_angle;
  private static boolean isLEDCall = false;
  private static boolean isDeliveryCall = false; 
  private static boolean isBasicCall = false; 
  private static ItemType m_desiredItem;

// For non-delivery use
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle) {
    m_elbow = elbow;
    m_angle = angle;
    
    // 0
   addRequirements(elbow);
  }
// For delivery buttons 
// When called, the command uses the current instance variable desired item to initialize the increment
// The purpose of the extra parameter-which is useless-is to call the special constructor
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle, boolean identifyAsDelivery) {
    m_elbow = elbow;
    m_angle = angle;
      
    addRequirements(elbow);
  }
  // For the LED buttons
  // When called, it updates and uses a new item type for the increment
  // The extra parameter is an enum to save and apply the item type
  public SetArmElbowCommand(ArmElbowSubsystem elbow, ItemType item) {
  m_elbow = elbow;
  m_angle = m_elbow.getDesiredAngle();
  m_desiredItem = item;

  isLEDCall = true;

  addRequirements(elbow);

  }

  @Override
  public void initialize() {
   double incrementAngle = 0;

  if(isLEDCall) {
  m_elbow.setDesiredItem(m_desiredItem);
  this.m_angle = m_elbow.getDesiredAngle();
  switch(m_elbow.getDesiredItem()) {
    case CUBE:
    incrementAngle = ArmConstants.kCubeAngleIncrement; // 10
    break;
    case CONE:
    incrementAngle = ArmConstants.kConeAngleIncrement; // 0
    break;
    }
}

if(isDeliveryCall) {
  switch(m_elbow.getDesiredItem()) {
    case CUBE:
    incrementAngle = ArmConstants.kCubeAngleIncrement; // -10
    break;
    case CONE:
    incrementAngle = ArmConstants.kConeAngleIncrement; // 0
    break;
        }
}

if(isBasicCall) {
  incrementAngle = ArmConstants.kConeAngleIncrement;
}

  m_elbow.setAngle(m_angle + incrementAngle);

  }

}
