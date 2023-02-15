package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
    // Swerve driveTrain subsystem constants
    public static final class SwerveDriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 51;
        public static final int kRearLeftDrivingCanId = 52;
        public static final int kFrontRightDrivingCanId = 54;
        public static final int kRearRightDrivingCanId = 53;
    
        public static final int kFrontLeftTurningCanId = 41;
        public static final int kRearLeftTurningCanId = 42;
        public static final int kFrontRightTurningCanId = 44;
        public static final int kRearRightTurningCanId = 43;
    
        public static final boolean kGyroReversed = false;
      }
    
      // Swerve Module Constants
    public static final class SwerveModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 13;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = (3 /*inches*/ * (2.54 / 100));
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class SwerveDriveAutonomousConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    // Ensign driveTrain (AM14U) subsystem constants
    public final static class KitbotDriveTrain {
        // CAN Bus addresses for motors
        public static int kCANMotorFL = 10;  // USING Victor SPX
        public static int kCANMotorRL = 11;  // USING Victor SPX
        public static int kCANMotorFR = 22;  // USING Victor SPX
        public static int kCANMotorRR = 7;  // USING Victor SPX
     

        public static boolean kLeftMotorsInverted = false;
        public static boolean kRightMotorsInverted = false;
        
        // Encoder Ports
        public static int[] kLeftEncoderPorts = {0,1};
        public static int[] kRightEncoderPorts = {2,3};

        public static boolean kLeftEncoderReversed = true;
        public static boolean kRightEncoderReversed = false;

        // Power Multiplicators
        public static double kPowerRatioLowSpeed = 0.5;
        public static double kPowerRatioHighSpeed = 1.0;

        // Drivetrain Characteristics
        public static double kWheelDiameter = Units.inchesToMeters(6.0) ;
        public static int kEncoderPulsesPerRev = 8192; // Through bore Encoder
        public static double kEncoderMetersPerPulse = kWheelDiameter * Math.PI / kEncoderPulsesPerRev;
        public static final double kTrackwidthMeters = Units.inchesToMeters(21.75);

        // TODO update placeholder values _ NEED TO DO CHARACTERIZATION
        public static final double ks = 1;
        public static final double kv = 3;

        // public static final double ksVolts = 0.22;
        // public static final double kvVoltSecondsPerMeter = 1.98;
        // public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        // public static final double kMaxAvailableVoltage = 10.5; // Assumes Battery "sag" for PID/Ramsete Controllers
        // public static final double kPDriveVel = 8.5;

        // public static final double kMaxSpeedMetersPerSecond = 3;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;    }

    // Elevator subsystem constants
    public final static class Elevator {
    }
    
    // Arm subsystem constants
    public final static class ArmConstants {
        public static final int kElbowCanId = 8;
        public static final boolean kElbowEncoderInverted = false;

        public static final double kElbowEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kElbowEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kElbowEncoderPositionPIDMinInput = 0; // radians
        public static final double kElbowEncoderPositionPIDMaxInput = kElbowEncoderPositionFactor; // radians
    
        public static final double kElbowP = 1;
        public static final double kElbowI = 0;
        public static final double kElbowD = 0;
        public static final double kElbowFF = 0;
        public static final double kElbowMinOutput = -1;
        public static final double kElbowMaxOutput = 1;
    
        
        public static final IdleMode kElbowMotorIdleMode = IdleMode.kBrake;
        public static final int kElbowMotorCurrentLimit = 40; // amps
        
    }
    
    // Claw subsystem constants
    public final static class Claw {
        // RoboRio/Can addresses
        public static int kCANMotorClaw=1; // Talon SPX

        public static int kDIOLimitSwitch = 5; // DIO
        public static int kDIOEncoderA = 6;    // DIO
        public static int kDIOEncoderB = 7;    // DIO
                
        // Settings
        public static double kTicksPerRev = 7.0 * 40.0 * (50/14); // quadrature cycles - 7 on motor, 40:1 for gearbox, Claw Gears: 50:14
       
        // TODO: These next two need to be calibrated
        public static int kSoftCloseAngle = 10; // Degrees
        public static int kHardCloseAngle = 20; // Degrees

        // PID Controller Constants
        public static double kP = 1.0;    
        public static double kI = 0.0;    
        public static double kD = 0.0;    
    }
    
    // LEDManager constants
    public final static class LEDManager {
    }

    // IO Controls
    public final static class IOControls {
        public static final int kXboxControllerPort = 0;
        public static final int kJoystickControllerPort = 1;    
        public static final double kDriveDeadband = 0.05;
    }


}

