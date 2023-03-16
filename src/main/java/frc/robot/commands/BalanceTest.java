// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BalanceTest extends PIDCommand {

  /** Creates a new BalanceCommandRemastered. */
  public BalanceTest(SwerveDriveSubsystem drivetrain) {
    super(
        // The controller that the command will use
        //0.005 P 0.00001 D
        new PIDController(0.005, 0, 0.00001),
        // This should return the measurement
        () -> drivetrain.getPitch(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drivetrain.drive(-output, 0, 0, false, true);
        });
  
        addRequirements(drivetrain);
  }

  
}
