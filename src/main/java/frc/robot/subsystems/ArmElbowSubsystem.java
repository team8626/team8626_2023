// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElbowSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private final WPI_VictorSPX m_elbowMotor = new WPI_VictorSPX(38);
  // TODO
  // Declare our Sensor(s)
  // TODO
  private final Encoder m_elbowEncoder = new Encoder(21, 22);
  /** Class Constructor. */
  public ArmElbowSubsystem() {
    m_elbowMotor.setInverted(false);
  }
  public void setMotor(double speed){
    m_elbowMotor.set(speed);
  }
  @Override
  public void periodic() {
  }
}

