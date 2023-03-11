// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Team8626 Libraries
import frc.robot.DashBoard.Trajectory;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class Autonomous {

    private final DashBoard m_dashboard;
    private final SubsystemBase m_drivetrain;


    private Trajectory m_autoStart;

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
    public Command getStartCommand(SwerveDriveSubsystem drive, HashMap<String, Command> map) throws IOException {
        SwerveDriveSubsystem m_drive = drive;
        HashMap<String, Command> m_map = map;
        m_autoStart = m_dashboard.getAutoSelection();
        String path = null;

       //  BOTTOM_DELIVER, MIDDLE_DELIVER_BALANCE, TOP_DELIVER
switch(m_autoStart) {

case BOTTOM_DELIVER: 
path = "BottomExit_Deliver";
break;

case TOP_DELIVER:
path = "TopExit_Deliver";
break;

case MIDDLE_DELIVER_BALANCE:
path = "MiddleExit_Deliver_Balance";
path = "MiddleExit_Deliver_Reverse_Balance";
break;

}

//  Load trajectory file "Example Path.path" and generate it with a max velocity of 1 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, new PathConstraints(1.0, 3.0));

    return new FollowPathWithEvents(
      m_drive.followTrajectoryCommand(trajectory, true),
      trajectory.getMarkers(),
      m_map);
        
    }  
}
