// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashBoard {

    enum TrajectoryEnum  {
        // BOTTOM_DELIVER, MIDDLE_DELIVER_BALANCE, TOP_DELIVER,
        REVERE_BALANCE,
        KICK_REVERE_BALANCE,
        KICK_EXIT_REVERE_BALANCE,
        DELIVER_STAY,

        START9_EXIT,
        START9_EXIT_BALANCE,
        START9_EXIT_BALANCE_STRAIGHT,

        // START9_CONE6_BALANCE, 
        // START9_CONE7_BALANCE, 

        START6_EXIT_BALANCE,

        START1_EXIT,
        DELIVER_EXIT_STRAIGHT,
        START1_EXIT_BALANCE,

        DO_NOTHING
    }

    private static RobotContainer m_robotContainer;
    private final Notifier m_thread = new Notifier(new dashboardThread());
	public static final boolean kEnableDashBoard = true;
	
    // private DesiredMode mCachedDesiredMode = null;
    // private StartingPosition mCachedStartingPosition = null;
    SendableChooser<TrajectoryEnum> m_autonomousModeChooser = new SendableChooser<>();
    

    // SimpleWidget m_balanceMultiplier = new SimpleWidget();

    static final double kShortInterval = .02;
    static final double kLongInterval  = .5;
    
    static double m_shortOldTime = 0.0;
    static double m_longOldTime  = 0.0;   

    /**
     * Class Constructor
     * Initialize the Dashboard with defaul values of "settable" inputs.
     */
    public DashBoard(RobotContainer newRobotcontainer) {
        m_robotContainer = newRobotcontainer;
        if(kEnableDashBoard){
            initAutonomousStrategy();
        }
        m_thread.startPeriodic(kShortInterval);
        initSubsystems();
    }

    /** 
     * Initialize Robot Autonomous Strategy 
     */
    private void initAutonomousStrategy(){
        // m_autonomousModeChooser.addOption("Start9 - Cone6 - Balance", TrajectoryEnum.START9_CONE6_BALANCE);
        // m_autonomousModeChooser.addOption("Start9 - Cone7 - Balance", TrajectoryEnum.START9_CONE7_BALANCE);
        m_autonomousModeChooser.addOption("Start9 - Exit", TrajectoryEnum.START9_EXIT);
        m_autonomousModeChooser.addOption("Start9 - Exit - Balance", TrajectoryEnum.START9_EXIT_BALANCE);
        m_autonomousModeChooser.addOption("Start9 - Exit - Balance __STRAIGHT__", TrajectoryEnum.START9_EXIT_BALANCE_STRAIGHT);

        //m_autonomousModeChooser.addOption("Start6 - Exit - Balance", TrajectoryEnum.START6_EXIT_BALANCE);
    
        m_autonomousModeChooser.addOption("Start1 - Exit", TrajectoryEnum.START1_EXIT);
        m_autonomousModeChooser.addOption("Start1 - Exit - Balance", TrajectoryEnum.START1_EXIT_BALANCE);
        // m_autonomousModeChooser.addOption("TEST - 2m Intake", TrajectoryEnum.TWO_M_INTAKE);

        m_autonomousModeChooser.addOption("Revere Balance", TrajectoryEnum.REVERE_BALANCE);
        m_autonomousModeChooser.setDefaultOption("Kick - Revere Balance", TrajectoryEnum.KICK_REVERE_BALANCE);
        m_autonomousModeChooser.addOption("Kick - Exit - Balance", TrajectoryEnum.KICK_EXIT_REVERE_BALANCE);
        m_autonomousModeChooser.addOption("Deliver - Stay", TrajectoryEnum.DELIVER_STAY);
        m_autonomousModeChooser.addOption("Deliver - Exit Straight", TrajectoryEnum.DELIVER_EXIT_STRAIGHT);
        m_autonomousModeChooser.addOption("Do Nothing", TrajectoryEnum.DO_NOTHING);

        SmartDashboard.putData("Auto Mode", m_autonomousModeChooser);
    }

    /** 
     * Returns Selected Robot Autonomous Startup Mode
     */  
    public TrajectoryEnum getAutoSelection() {
        return m_autonomousModeChooser.getSelected();
    }

    private static void updateDashboard() {
        double time = Timer.getFPGATimestamp();
        if (kEnableDashBoard) {
            if ((time - m_shortOldTime) > kShortInterval) {
                m_shortOldTime = time;
                updateShortInterval();
            }
            if ((time - m_longOldTime) > kLongInterval) {
                //Thing that should be updated every LONG_DELAY
                m_longOldTime = time;
                updateShortInterval();
                updateLongInterval();
            }
        }
    }


    // Initialize Dashboard for all subsystems.
    private void initSubsystems() {
        m_robotContainer.m_pneumatic.initDashboard();
        m_robotContainer.m_claw.initDashboard();
        m_robotContainer.m_elbow.initDashboard();
        m_robotContainer.m_elevator.initDashboard();
        m_robotContainer.m_extender.initDashboard();
        m_robotContainer.m_drive.initDashboard();
    }

    // Update values that need high frequency refresh.
    private static void updateShortInterval() {
        // Pulsing to indicate Dashboard is updated
        dashboardFlash();
        

        //  m_elevator.updateDashboard();
        m_robotContainer.m_pneumatic.updateDashboard();
        m_robotContainer.m_claw.updateDashboard();
        m_robotContainer.m_elbow.updateDashboard();
        m_robotContainer.m_elevator.updateDashboard();
        m_robotContainer.m_extender.updateDashboard();
        m_robotContainer.m_drive.updateDashboard();
    }

    // Update values that need low frequency refresh.
    private static void updateLongInterval(){
     
    }

    //Flash a light on the dashboard so that you know that the dashboard is refreshing.
    static int t = 0;
    static boolean b = true;
    public static void dashboardFlash(){
        if (t > 20) {
            t = 0;
            b = !b;
            SmartDashboard.putBoolean("Pulse", b);
        }
        t++;
    }

    private static class dashboardThread implements Runnable {  
        @Override
        public void run() {
            updateDashboard();
        }
    }

}
