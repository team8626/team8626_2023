// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;;


public class ArmElbowSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private final CANSparkMax m_elbowMotor;

  // Declare our Sensor(s)
  private final AbsoluteEncoder m_elbowEncoder;
  private final SparkMaxPIDController m_elbowPIDController;


  private double m_desiredAngle = 0.0;

  /** Class Constructor. */
  public ArmElbowSubsystem() {

     m_elbowMotor = new CANSparkMax(ArmConstants.kElbowCanId, MotorType.kBrushless);

    // Factory Reset
    m_elbowMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for SPARKMAX.
    m_elbowEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_elbowPIDController = m_elbowMotor.getPIDController();
    m_elbowPIDController.setFeedbackDevice(m_elbowEncoder);
    
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in degrees and degrees per second
    m_elbowEncoder.setPositionConversionFactor(ArmConstants.kElbowEncoderPositionFactor);
    m_elbowEncoder.setVelocityConversionFactor(ArmConstants.kElbowEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_elbowEncoder.setInverted(ArmConstants.kElbowEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
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

    m_elbowMotor.setIdleMode(ArmConstants.kElbowMotorIdleMode);
    m_elbowMotor.setSmartCurrentLimit(ArmConstants.kElbowMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_elbowMotor.burnFlash();

    m_desiredAngle = m_elbowEncoder.getPosition();
  }

  /**
   * Returns the current state of the module.
   */
  public double getAngle() {
    return m_elbowEncoder.getPosition();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setAngle(double desiredAngle) {
    m_elbowPIDController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition);
    m_desiredAngle = desiredAngle;
  }

  @Override
  public void periodic() {
  }

  public void initDashboard(){

  }

  public void updateDashboard(){
    
  }
}




