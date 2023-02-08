// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Team8626 Libraries
import frc.robot.DashBoard.AutoSelec;
import frc.robot.DashBoard.StartPosition;
import frc.robot.subsystems.DriveSubsystem;

public class Autonomous {

    private final DashBoard m_dashboard;
    private final SubsystemBase m_drivetrain;

    public StartPosition m_startPosition;
    public AutoSelec m_autoStart;

    public Autonomous(DashBoard dashboard, SubsystemBase drivetrain){
        m_dashboard = dashboard;
        m_drivetrain = drivetrain;
       
    }
/* enum StartPosition {
        LEFT_SIDE, RIGHT_SIDE, MIDDLE_SIDE
    }

    enum AutoSelec {
       EXIT, EXIT_BALANCE, NODE_EXIT_BALANCE
    } */
    public Command getStartCommand() throws IOException {
        Command startCommand = null;
        m_startPosition = m_dashboard.getStartPosition(); 
        m_autoStart = m_dashboard.getAutoMode();
         //   switch(m_startPosition) {}
        switch (m_autoStart) {
case EXIT:
switch(m_startPosition) {
    case LEFT_SIDE:
startCommand = new PrintCommand("Exiting Tarmac from the left side");
    break;
    case MIDDLE_SIDE:
    startCommand = new PrintCommand("Exiting Tarmac from the middle");
    break;
    case RIGHT_SIDE:
    startCommand = new PrintCommand("Exiting Tarmac from the right side");
    break;
}
break;
case EXIT_BALANCE:
switch(m_startPosition) {
    case LEFT_SIDE:
startCommand = new PrintCommand("Exiting Tarmac from the left side and balancing");
    break;
    case MIDDLE_SIDE:
    startCommand = new PrintCommand("Exiting Tarmac from the middle and balancing");
    break;
    case RIGHT_SIDE:
    startCommand = new PrintCommand("Exiting Tarmac from the right side and balancing");
    break;
}
break;
case NODE_EXIT_BALANCE:
switch(m_startPosition) {
    case LEFT_SIDE:
startCommand = new PrintCommand("Delivering node, exiting Tarmac from the left side, and balancing");
    break;
    case MIDDLE_SIDE:
    startCommand = new PrintCommand("Delivering node, exiting Tarmac from the middle, and balancing");
    break;
    case RIGHT_SIDE:
    startCommand = new PrintCommand("Delivering node, exiting Tarmac from the right side, and balancing");
    break;
}
break; 
        }
        return startCommand;
    }  
}
