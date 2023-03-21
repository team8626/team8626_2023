// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

// WPI Libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// Team8626 Libraries
import frc.robot.DashBoard.TrajectoryEnum;
import frc.robot.commands.BalanceTest;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.SetStowPositionCommand;
import frc.robot.commands.auto.ReadyForGrid2;

public class Autonomous {

    private final DashBoard m_dashboard;
    private RobotContainer m_robot;

    private TrajectoryEnum m_autoStart;

    public Autonomous(DashBoard dashboard, SubsystemBase drivetrain){
        m_dashboard = dashboard;
       
    }

    public Command getStartCommand(RobotContainer newRobotContainer) throws IOException {
        m_autoStart = m_dashboard.getAutoSelection();
        m_robot = newRobotContainer;

        Command startCommand = new InstantCommand();

        switch(m_autoStart) {
  
            case START9_CONE6_BALANCE:
                startCommand = getStart9Cone6BalanceCommand();
                break;
            case START9_CONE7_BALANCE:
                startCommand = getStart9Cone7BalanceCommand();
                break;
            case START6_EXIT_BALANCE:
                startCommand = getStart6ExitBalanceCommand();
                break;
            case START9_EXIT:
                startCommand = getStart9ExitCommand();
                break;
            case START1_EXIT:
                startCommand = getStart1ExitCommand();
                break;
            case TWO_M_INTAKE:
                startCommand = getTwoMeterIntakeCommand();
                break;
            case TWO_M_BALANCE:
                startCommand = getTwoMeterBalanceCommand();
                break;  
            case DO_NOTHING:
                startCommand = new InstantCommand();
        }
        return startCommand;
    }

    private Command getStart9Cone7BalanceCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start9_Cone4_Balance", new PathConstraints(1.5, 3));

        startCommand = new SequentialCommandGroup(
            // Starting the game. Make sure the claw is closed and get ready for delivery
            new CloseClawCommand(m_robot.m_claw),
            new ReadyForGrid2(m_robot.m_elevator, m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_ledManager),
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Go to pickup next piece
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap),

            new CloseClawCommand(m_robot.m_claw),
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(1), false),
                                    pathGroup.get(1).getMarkers(),
                                    m_robot.eventMap),

            // At this point, we should be ready to deliver
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Go to Charging Station
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(2), false),
                                    pathGroup.get(2).getMarkers(),
                                    m_robot.eventMap),
            new PrintCommand("---------- AUTO Ready to Balance ----------"),
            new BalanceTest(m_robot.m_drive, m_robot.m_ledManager));

        return startCommand;
    }

    private Command getStart9Cone6BalanceCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start9_Cone6_Balance", new PathConstraints(1.5, 3));

        startCommand = new SequentialCommandGroup(
            // Starting the game. Make sure the claw is closed and get ready for delivery
            new CloseClawCommand(m_robot.m_claw),
            new ReadyForGrid2(m_robot.m_elevator, m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_ledManager),
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Go to pickup next piece
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap),

            new CloseClawCommand(m_robot.m_claw),
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(1), false),
                                    pathGroup.get(1).getMarkers(),
                                    m_robot.eventMap),

            // At this point, we should be ready to deliver
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Go to Charging Station
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(2), false),
                                    pathGroup.get(2).getMarkers(),
                                    m_robot.eventMap),
            new PrintCommand("---------- AUTO Ready to Balance ----------"),
            new BalanceTest(m_robot.m_drive, m_robot.m_ledManager));

        return startCommand;
    }

    private Command getStart6ExitBalanceCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start6_Exit_Balance", new PathConstraints(1, 3));

        startCommand = new SequentialCommandGroup(
            // Starting the game. Make sure the claw is closed and get ready for delivery
            new CloseClawCommand(m_robot.m_claw),
            new ReadyForGrid2(m_robot.m_elevator, m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_ledManager),
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Exit the Community & Comme Back for Balancing
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap),
            new PrintCommand("---------- AUTO Ready to Balance ----------"),
            new BalanceTest(m_robot.m_drive, m_robot.m_ledManager));

        return startCommand;
    }

    private Command getStart9ExitCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start9_Exit", new PathConstraints(1, 3));

        startCommand = new SequentialCommandGroup(
            // Starting the game. Make sure the claw is closed and get ready for delivery
            new CloseClawCommand(m_robot.m_claw),
            new ReadyForGrid2(m_robot.m_elevator, m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_ledManager),
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Exit the Community
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap));

        return startCommand;
    }

    private Command getStart1ExitCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Start1_Exit", new PathConstraints(1, 3));

        startCommand = new SequentialCommandGroup(
            // Starting the game. Make sure the claw is closed and get ready for delivery
            new CloseClawCommand(m_robot.m_claw),
            new ReadyForGrid2(m_robot.m_elevator, m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_ledManager),
            new WaitCommand(.25),
            new OpenClawCommand(m_robot.m_claw),

            // Go to pickup next piece
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap));

        return startCommand;
    }


    private Command getTwoMeterIntakeCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test_2m_Forward", new PathConstraints(1, 3));

        startCommand = new SequentialCommandGroup(
            // Test Command for intaking a game piece at 2meters away
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    m_robot.eventMap),
            new CloseClawCommand(m_robot.m_claw),
            new SetStowPositionCommand(m_robot.m_elbow, m_robot.m_extender, m_robot.m_claw, m_robot.m_elevator, m_robot.m_ledManager) );

        return startCommand;
    }
    
    private Command getTwoMeterBalanceCommand(){
        Command startCommand = new InstantCommand();

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test_2m_Forward", new PathConstraints(1, 3));

        startCommand = new SequentialCommandGroup(
            // Test Command for balancing after moving 2 meters
            new FollowPathWithEvents(m_robot.m_drive.followTrajectoryCommand( pathGroup.get(0), true),
                                    pathGroup.get(0).getMarkers(),
                                    new HashMap<>()),
            new PrintCommand("---------- AUTO Ready to Balance ----------"),
            new BalanceTest(m_robot.m_drive, m_robot.m_ledManager));

        return startCommand;
    }
}