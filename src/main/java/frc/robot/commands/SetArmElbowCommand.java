// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.subsystems.ArmElbowSubsystem;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.ArmElbowSubsystem.ItemType;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class SetArmElbowCommand extends InstantCommand {
  private final ArmElbowSubsystem m_elbow;
  private final LEDManagerSubsystem m_ledManager;
  private double m_angle;
  private boolean m_isLEDCall = false;
  private boolean m_isDeliveryCall = false; 
  private boolean m_isBasicCall = false; 
  private ItemType m_newDesiredItem;

// For non-delivery use
  public SetArmElbowCommand(ArmElbowSubsystem elbow, LEDManagerSubsystem LEDManager, double angle) {
    m_elbow = elbow;
    m_ledManager = LEDManager;
    m_angle = angle;

    m_isBasicCall = true;
    m_isDeliveryCall = false;
    m_isLEDCall = false;
    addRequirements(elbow);
  }
// For delivery buttons 
// When called, the command uses the current instance variable desired item to initialize the increment
// The purpose of the extra parameter-which is useless-is to call the special constructor
  public SetArmElbowCommand(ArmElbowSubsystem elbow, LEDManagerSubsystem LEDManager, double angle, boolean identifyAsDelivery) {
    m_elbow = elbow;
    m_ledManager = LEDManager;
    m_angle = angle;

    

    m_isBasicCall = false;
    m_isDeliveryCall = true;
    m_isLEDCall = false;
    addRequirements(elbow);
  }
  // For the LED buttons
  // When called, it updates and uses a new item type for the increment
  // The extra parameter is an enum to save and apply the item type
  public SetArmElbowCommand(ArmElbowSubsystem elbow, LEDManagerSubsystem LEDManager, ItemType newItem) {
    m_elbow = elbow;
    m_ledManager = LEDManager;

    m_angle = m_elbow.getDesiredAngle();
    m_newDesiredItem = newItem;

    m_isBasicCall = true;
    m_isDeliveryCall = false;
    m_isLEDCall = true;
    addRequirements(elbow);
  }

  @Override
  public void initialize() {
    double incrementAngle = 0.0;
    
    // LED Call, we adjust angle based on Item Type
    // -- Higher for cones (Smaller Angle)
    // -- Lower for Cubes (Larger Angle)
    if(m_isLEDCall) {
      // If requested Item is differenc, adjust the angle
      m_angle = m_elbow.getDesiredAngle();
      if(m_newDesiredItem != m_elbow.getDesiredItem()){
        switch(m_elbow.getDesiredItem()) {
          case CUBE: // Currently set to a cube, new item will be a cone ==> Higher ==> Lower Angle
            incrementAngle = -ArmConstants.kCubeAngleIncrement; // -10
            m_ledManager.setColor(LEDManagerConstants.kColorCONE);
            System.out.printf("[SetArmElbowCommand] Changing to Cone\n"); 
            break;
          case CONE: // Currently set to a cone, new item will be a cube ==> Lower ==> Higher Angle
            incrementAngle = +ArmConstants.kCubeAngleIncrement; // +10
            System.out.printf("[SetArmElbowCommand] Changing to Cube\n"); 
            m_ledManager.setColor(LEDManagerConstants.kColorCUBE);
            break;
          case NONE:
            incrementAngle = 0;
            m_ledManager.setAllianceColor();
            m_newDesiredItem = ItemType.CONE;
        }
        m_elbow.setDesiredItem(m_newDesiredItem);
      }
      else {
          System.out.printf("[SetArmElbowCommand] Same Item - Doing Nothing\n"); 
      }
    } else if(m_isDeliveryCall) {
      System.out.printf("[SetArmElbowCommand] Delivery Call\n"); 
      switch(m_elbow.getDesiredItem()) {
        case CUBE: // Setting up for Cube delivery, adjust angle (We received Cone angle)
          incrementAngle = +ArmConstants.kCubeAngleIncrement; // +10
          m_ledManager.setColor(LEDManagerConstants.kColorCUBE);
          System.out.printf("[SetArmElbowCommand] Delivery Call: Cube\n"); 
      break;
        case CONE: // Setting up for Cone delivery, no need to adjust angle (We received Cone angle)
          incrementAngle = 0;
          System.out.printf("[SetArmElbowCommand] Delivery Call: Cone\n");
          m_ledManager.setColor(LEDManagerConstants.kColorCONE);
          break;
        case NONE:
          incrementAngle = 0;
          System.out.printf("[SetArmElbowCommand] This is not an item... Do nothing\n"); 
          break;
      }
      m_elbow.setDeliveryStatus(true);
    }
    // Basic Call (not delivery... Do nothing to adjust)
    else if(m_isBasicCall) {
      System.out.printf("[SetArmElbowCommand] Basic Call\n"); 
      incrementAngle = 0;
      m_elbow.setDeliveryStatus(false);
    }

    m_elbow.setAngle(m_angle + (m_elbow.isSetDelivery()? incrementAngle : 0));
  }
}
