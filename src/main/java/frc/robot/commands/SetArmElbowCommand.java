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
  private final double m_angle;
  private static double incrementAngle;
  
// For non-delivery use
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle) {
    m_elbow = elbow;
    m_angle = angle;
    
   incrementAngle = ArmConstants.kConeAngleIncrement; // 0
   addRequirements(elbow);
  }
// For delivery buttons 
// When called, the command uses the current instance variable desired item to initialize the increment
// The purpose of the extra parameter-which is useless-is to call the special constructor
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle, byte identifyAsDelivery) {
    m_elbow = elbow;
    m_angle = angle;
    
    switch(m_elbow.getDesiredItem()) {
      case CUBE:
      incrementAngle = ArmConstants.kCubeAngleIncrement; // -10
      break;
      case CONE:
      incrementAngle = ArmConstants.kConeAngleIncrement; // 0
      break;
          }
    addRequirements(elbow);
  }
  // For the LED buttons
  // When called, it updates and uses a new item type for the increment
  // The extra parameter is an enum to save and apply the item type
  public SetArmElbowCommand(ArmElbowSubsystem elbow, double angle, ItemType item) {
    m_elbow = elbow;
    m_angle = angle;

    m_elbow.setDesiredItem(item);

    switch(item) {
    case CUBE:
    incrementAngle = ArmConstants.kCubeAngleIncrement; // -10
    break;
    case CONE:
    incrementAngle = ArmConstants.kConeAngleIncrement; // 0
    break;
    }

    addRequirements(elbow);
  }

  @Override
  public void initialize() {
  m_elbow.setAngle(m_angle + incrementAngle);
  }

}
