package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveSubsystem extends SubsystemBase{

    // public boolean isSwerve(){
    //     return false;
    // }
    
    // public boolean isKitBot(){
    //     return false;
    // }

    public abstract boolean isSwerve();
    public abstract boolean isKitBot();

    public void setX(){};

    // Drive Command for Swerve
    //public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){};

    // Drive Command for KitBot
    //public void drive(double xSpeed, double rot){};
    
}
