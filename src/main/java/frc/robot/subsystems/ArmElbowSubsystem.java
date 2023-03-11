// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmElbowSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private final CANSparkMax m_elbowMotor;

  // Declare our Sensor(s)
  private final AbsoluteEncoder m_elbowEncoder;
  private final SparkMaxPIDController m_elbowPIDController;

private boolean m_setDelivery;
  private double m_desiredAngle = 359.00;

  public enum ItemType {
    CUBE, CONE, NONE
    }

  public ItemType m_desiredItem;

  /** Class Constructor. */
  public ArmElbowSubsystem() {
  m_desiredItem = ItemType.CONE;

     m_elbowMotor = new CANSparkMax(ArmConstants.kCANElbow, MotorType.kBrushless);

    // Factory Reset
    m_elbowMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for SPARKMAX.
    m_elbowEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_elbowPIDController = m_elbowMotor.getPIDController();
    m_elbowPIDController.setFeedbackDevice(m_elbowEncoder);


    // TODO: Set Limit Switches
    m_elbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
     m_elbowMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
     m_elbowMotor.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.kSoftLimitBottom);
     m_elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.kSoftLimitTop);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in degrees and degrees per second
    m_elbowEncoder.setPositionConversionFactor(ArmConstants.kElbowEncoderPositionFactor);
    m_elbowEncoder.setVelocityConversionFactor(ArmConstants.kElbowEncoderVelocityFactor);

    m_elbowEncoder.setInverted(ArmConstants.kElbowEncoderInverted);
    m_elbowPIDController.setPositionPIDWrappingEnabled(false);
    // m_elbowPIDController.setPositionPIDWrappingMinInput(ArmConstants.kElbowEncoderPositionPIDMinInput);
    // m_elbowPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kElbowEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_elbowPIDController.setP(ArmConstants.kElbowP);
    m_elbowPIDController.setI(ArmConstants.kElbowI);
    m_elbowPIDController.setD(ArmConstants.kElbowD);
    m_elbowPIDController.setFF(ArmConstants.kElbowFF);
    m_elbowPIDController.setOutputRange(ArmConstants.kElbowMinOutput, ArmConstants.kElbowMaxOutput);

    // TODO: Try this to =slow down arm going down
    //m_elbowPIDController.setSmartMotionMaxVelocity(10, 0); // Max Speed: 10 degreres per second

    m_elbowMotor.setIdleMode(ArmConstants.kElbowMotorIdleMode);
    m_elbowMotor.setSmartCurrentLimit(ArmConstants.kElbowMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_elbowMotor.burnFlash();

    // Set initial angle to the current position of the elbow.
    // (keep it there)
    m_desiredAngle = getAngle();
    setAngle(m_desiredAngle);
  }

  /**
   * Returns the current state of the module.
   */
  public double getAngle() {
    return m_elbowEncoder.getPosition();
  }

  public void setP(double kP){
    m_elbowPIDController.setP(kP);
  }
  public void setI(double kI){
    m_elbowPIDController.setP(kI);
  }
  public void setD(double kD){
    m_elbowPIDController.setP(kD);
  }
  
  /**
   * Sets the desired state for the module.
   *
   * @param desiredAngle Desired state with speed and angle.
   */
  public void setAngle(double desiredAngle) {
    m_elbowPIDController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition);
    m_desiredAngle = desiredAngle;
  }

  public double getDesiredAngle() {
    return m_desiredAngle;
  }

  public ItemType getDesiredItem() {
    return m_desiredItem;
  }

  @Override
  public void periodic() {
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Elbow Angle", getAngle());
    SmartDashboard.putNumber("Desired Angle", m_desiredAngle);
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Elbow Angle", getAngle());
    SmartDashboard.putNumber("Desired Angle", m_desiredAngle);
  }

  public void setDesiredItem(ItemType updatedItemType) {
    m_desiredItem = updatedItemType;
  }
  public void setDesiredAngle(double angle) {
    m_desiredAngle = angle;
  }

public boolean isSetDelivery() {
  return m_setDelivery;
}

public void setDeliveryStatus(boolean isSetDelivery) {
  m_setDelivery = isSetDelivery;
}

}





