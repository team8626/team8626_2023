package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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

        public static final double kLowestSpeedFactor = 0.3;
        public static final double kLowSpeedFactor = 0.5;
        public static final double kBalancedPositionTolerance = 2.5;
        public static final double kBalancedVelocityTolerance = 1;
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

    // Elevator subsystem constants
    public final static class ElevatorConstants {
        public static final int kCANElevator = 9; // SPARKMAX
        public static int kDIOLimitSwitchBottom = 8; // DIO
        public static int kDIOLimitSwitchTop = 9;    // DIO

    }
    
    // Arm subsystem constants
    public final static class ArmConstants {
        public static final int kCANElbow = 8; // SPARKMAX
        public static int kSolenoidChannel = 0; 

        public static final boolean kElbowEncoderInverted = false;

        public static final double kElbowEncoderPositionFactor = 360; // degrees
        public static final double kElbowEncoderVelocityFactor = 360 / 60.0; // degrees per second

        public static final double kElbowEncoderPositionPIDMinInput = 0; // degrees
        public static final double kElbowEncoderPositionPIDMaxInput = kElbowEncoderPositionFactor; // degrees
    
        public static final double kElbowP = 0.02;
        public static final double kElbowI = 0;
        public static final double kElbowD = 0;
        public static final double kElbowFF = 0;
        public static final double kElbowMinOutput = -.6; // Upwards
        public static final double kElbowMaxOutput = 0.1; // Downwards
    
        
        public static final IdleMode kElbowMotorIdleMode = IdleMode.kBrake;
        public static final int kElbowMotorCurrentLimit = 40; // amps

        public static final double kStowedElbowAngle = 348;

        public static final double kTopGridElbowAngle = 240; 
        public static final double kMiddleGridElbowAngle = 260;
        public static final double kBottomGridElbowAngle = 300;
        public static final double kLockArmElbowAngle = 355;

        public static final double kSubstationlbowAngle = 260;
        public static final double kFloorElbowAngle = 295;

        public static final double kTraverseElbowAngle = 348;
        public static final double kCubeAngleIncrement = 15;
        public static final double kConeAngleIncrement = 0;

        public static final float kSoftLimitBottom = 359;
        public static final float kSoftLimitTop = 220;
        public static final double kMaxOpenClawAngle = 305;
       
    }
    
    public final static class PneumaticConstants {
        public static final double kMinClawPSI = 20;
        public static PneumaticsModuleType kPCMtype = PneumaticsModuleType.REVPH;
        public static double kMinPressure = 80;
        public static double kMaxPressure = 120;
        public static double kMinArmPSI = 20;
    }

    // Claw subsystem constants
    public final static class ClawConstants {
        // RoboRio/Can addresses
        public static int kCANMotorClaw = 20;  // Talon SPX
        public static int kSolenoidChannel = 3; 
    }

    // LEDManager constants
    public final static class LEDManagerConstants {
        public static int kDIOCom1 = 0; // DIO
        public static int kDIOCom2 = 1; // DIO
        public static int kDIOCom3 = 2; // DIO

        public static byte kColorOFF          = 0b000;
        public static byte kColorCONE         = 0b001;
        public static byte kColorCUBE         = 0b010;
        public static byte kColorALLIANCEBLUE = 0b011;
        public static byte kColorALLIANCERED  = 0b100;
        public static byte kColorPINK         = 0b110;
        public static byte kColorWHITE        = 0b101;
        public static byte kColorRAINBOW      = 0b111;
    }

    // IO Controls
    public final static class IOControlsConstants {
        public static final int kXboxControllerPort = 0;
        public static final int kJoystickControllerPort = 1;    
        public static final int kButtonBoxPort = 2;    
        public static final double kDriveDeadband = 0.05;
    }
}

