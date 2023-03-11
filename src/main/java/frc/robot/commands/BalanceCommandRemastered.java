// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.subsystems.LEDManagerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceCommandRemastered extends PIDCommand {
  private PIDController m_PID;
  private double minVelocity = 3;
  private double maxStillAngle = 10;
  private LEDManagerSubsystem m_ledManager;
  /** Creates a new BalanceCommandRemastered. */
  public BalanceCommandRemastered(SwerveDriveSubsystem drivetrain, LEDManagerSubsystem ledManager) {
    super(
        // The controller that the command will use
        new PIDController(0.02, 0, 0),
        // This should return the measurement
        () -> drivetrain.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drivetrain.drive(output, 0,0,false, true);
        });
        m_ledManager = ledManager;

        addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_PID = getController();
  }

 
/* 
We will fly if we can't manipulate the charge station
Keep in mind the error is based off of zero
*/

  @Override
  public void execute() {
    m_ledManager.setColor(LEDManagerConstants.kColorWHITE);

  // Check if we are not making progress 
  if((Math.abs(m_PID.getVelocityError()) < minVelocity) 
  // Checking angle, will cancel at rest without this
  && (Math.abs(m_PID.getPositionError()) > maxStillAngle)) 
    cancel();
    m_ledManager.setAllianceColor();
  }


}
