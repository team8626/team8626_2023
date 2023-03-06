// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
  // Declare our Motor(s)
  private WPI_VictorSPX m_motor = new WPI_VictorSPX(ClawConstants.kCANMotorClaw);

  // Declare and initialize our Sensor(s)
  private Encoder m_encoder = new Encoder(ClawConstants.kDIOEncoderA, ClawConstants.kDIOEncoderB, false);
  private DigitalInput m_openSwitch = new DigitalInput(ClawConstants.kDIOLimitSwitch);
  private boolean m_homed = false;
  
  // Simulation opbjects
  private DIOSim m_openSwitchSim = new DIOSim(m_openSwitch);
  private EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Class Constructor
  public ClawSubsystem() {
    m_encoder.setDistancePerPulse(360.0 / ClawConstants.kTicksPerRev); // Degrees per pulse

    // Set Values for starting simulation
    if(Robot.isSimulation()){
      m_openSwitchSim.setValue(false);
      m_encoderSim.setCount(1234);
    }
  }

  // Return angle of the claw
  // Returns -99.99 if system hasn't been homed
  public double getAngle() {
    double angle = -99.99;
    if(isHomed()){
      angle = m_encoder.getDistance();
    }
    return angle;
  }
  
  public void initDashboard(){
    SmartDashboard.putBoolean("Claw Homed", isHomed());
    SmartDashboard.putNumber("Claw Angle", getAngle());
  }

  public void updateDashboard(){
    SmartDashboard.putBoolean("Claw Homed", isHomed());
    SmartDashboard.putNumber("Claw Angle", getAngle());
  }

  public boolean isHomed() {
    return m_homed;
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
    setMotor(speed, false); 
  }

  public void setMotor(double speed, boolean forced) {
    // Not moving if not homed
    if(m_homed || forced){
      m_motor.set(speed);
      
      // Set Values for starting simulation
      if(Robot.isSimulation()){
        m_openSwitchSim.setValue(false);
        m_encoderSim.setCount((m_encoderSim.getCount() + 1 ));
      }
    }
  }
  public void stop() {
    m_motor.set(0.0);
  }

  @Override
  public void periodic() {
    if(Robot.isReal()){
      // If at limit switch, reset encoder and consider motor homed
      if(m_openSwitch.get() == true){
        stop();
        m_encoder.reset();
        m_homed = true;
      }
    } else {
      // Running Simulation
      if(m_openSwitchSim.getValue() == true){
        stop();
        m_encoder.reset();
        m_homed = true;
      }
    }
  }
}