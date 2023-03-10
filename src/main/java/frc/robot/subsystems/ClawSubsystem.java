// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private WPI_VictorSPX m_motor = new WPI_VictorSPX(ClawConstants.kCANMotorClaw);
  // Declare Pneumatic Solenoid
  private final DoubleSolenoid m_cylinder = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 2);

  // private final Solenoid m_cylinderOpen = new Solenoid(PneumaticsModuleType.REVPH, 0 /*8*/);
  // private final Solenoid m_cylinderClose = new Solenoid(PneumaticsModuleType.REVPH, 1 /*9*/);
  // Declare and initialize our Sensor(s)
  private Encoder m_encoder = new Encoder(ClawConstants.kDIOEncoderA, ClawConstants.kDIOEncoderB, false);
  private DigitalInput m_openSwitch = new DigitalInput(ClawConstants.kDIOLimitSwitch);
  private boolean m_isClosed = true;
  
  // Class Constructor
  public ClawSubsystem() {
    m_encoder.setDistancePerPulse(360.0 / ClawConstants.kTicksPerRev); // Degrees per pulse
    // m_cylinderOpen.set(true);
    // m_cylinderClose.set(false);
    m_cylinder.set(Value.kForward);
  }


  public void initDashboard(){
    SmartDashboard.putBoolean("Claw Homed", isHomed());
    SmartDashboard.putBoolean("Claw Closed", m_isClosed);
  }

  public void updateDashboard(){
    SmartDashboard.putBoolean("Claw Homed", isHomed());
    SmartDashboard.putBoolean("Claw Closed", !m_isClosed);
  }

  public boolean isHomed() {
    return true;
  }


  public boolean isOpened() {
    return m_openSwitch.get();
  }

  // Homing the subsystem.
  // Open Claw until Limiti Switch is hit
  public void homeSubsystem() {
      m_motor.set(-1.0);
  }

  public void setMotor(double speed) {
    m_motor.set(speed);
  }

  public void closePneumatic(){
    // m_cylinderOpen.set(false);
    // m_cylinderClose.set(true);
    m_cylinder.set(Value.kForward);


  }

  public void openPneumatic(){
    // m_cylinderOpen.set(true);
    // m_cylinderClose.set(false);
    m_cylinder.set(Value.kReverse);

  }

  public void stop() {
    setMotor(0);
  }





  @Override
  public void periodic() {
    // If at limit switch, reset encoder and consider motor homed
    if(m_openSwitch.get() == true){
      stop();
      m_encoder.reset();
    }
  }
}