// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private final CANSparkMax m_elevatorMotor  = new CANSparkMax(ElevatorConstants.kCANElevator, MotorType.kBrushless);

  // Declare our Sensor(s)
  private static DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.kDIOLimitSwitchTop);
  private static DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.kDIOLimitSwitchBottom);

  /** Class Constructor. */
  public ElevatorSubsystem() {
    m_elevatorMotor.restoreFactoryDefaults();
    m_elevatorMotor.setInverted(true);

  initDashboard();
  }

  public boolean getTopPressed() {
  return (topLimitSwitch.get());
  }
  
  public boolean getBottomPressed(){
    return (bottomLimitSwitch.get());
  }
  
  public static String getPosition(){
    if(bottomLimitSwitch.get()) return "BOTTOM";
    else if(topLimitSwitch.get()) return "TOP";
    else return "UNKNOWN";
  }
  
  public void setMotor(double speed) {
    m_elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
  }

  public void initDashboard() {
    SmartDashboard.putString("Elevator Position", "NOT INITIALIZED");
  }
  
  public void updateDashboard() {
    SmartDashboard.putString("Elevator Position", getPosition());
  }

}

