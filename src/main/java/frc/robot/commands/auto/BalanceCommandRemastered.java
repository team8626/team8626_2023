// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.LEDManagerConstants;
import frc.robot.Constants.SwerveDriveConstants;
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
        new PIDController(100, 0, 0),
        // This should return the measurement
        () -> drivetrain.getPitch(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drivetrain.drive(output, 0, 0, false, true);
        });
        m_ledManager = ledManager;

        addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_PID = getController();
    m_PID.setTolerance(SwerveDriveConstants.kBalancedPositionTolerance, SwerveDriveConstants.kBalancedVelocityTolerance);
  }

  @Override
  public void end(boolean interrupted) {
    
    System.out.println(" ______                  ______       _                  _             ");
    System.out.println("|  _  \\                 | ___ \\     | |                (_)            ");
    System.out.println("| | | |___  _ __   ___  | |_/ / __ _| | __ _ _ __   ___ _ _ __   __ _ ");
    System.out.println("| | | / _ \\| '_ \\ / _ \\ | ___ \\/ _` | |/ _` | '_ \\ / __| | '_ \\ / _` |");
    System.out.println("| |/ / (_) | | | |  __/ | |_/ / (_| | | (_| | | | | (__| | | | | (_| |");
    System.out.println("|___/ \\___/|_| |_|\\___| \\____/ \\__,_|_|\\__,_|_| |_|\\___|_|_| |_|\\__, |");
    System.out.println("                                                                 __/ |");
    System.out.println("                                                                |___/ ");

   // m_ledManager.setAllianceColor();
  }    

  @Override
  public void initialize() {
System.out.println(" ______       _                  _             ");
System.out.println("| ___ \\     | |                (_)            ");
System.out.println("| |_/ / __ _| | __ _ _ __   ___ _ _ __   __ _ ");
System.out.println("| ___ \\/ _` | |/ _` | '_ \\ / __| | '_ \\ / _` |");
System.out.println("| |_/ / (_| | | (_| | | | | (__| | | | | (_| |");
System.out.println("\\____/ \\__,_|_|\\__,_|_| |_|\\___|_|_| |_|\\__, |");
System.out.println("                                         __/ |");
System.out.println("                                        |___/ ");

m_ledManager.setColor(LEDManagerConstants.kColorCONE);

  }


/* 
We will fly if we can't manipulate the charge station
Keep in mind the error is based off of zero
*/

  @Override
  public void execute() {

  // Check if we are not making progress 
  if((Math.abs(m_PID.getVelocityError()) < minVelocity) 
  // Checking angle, will cancel at rest without this
  && (Math.abs(m_PID.getPositionError()) > maxStillAngle)) {
    System.out.println("Trying to cancel");
      // cancel();
    }
    // 
    if(m_controller.atSetpoint()) {
      // new UpdateLEDsCommand(m_ledManager, LEDManagerConstants.kColorCUBE).schedule();
    } 
    else {
     // new SetAllianceColorCommand(m_ledManager).schedule();
    }
  }
}
