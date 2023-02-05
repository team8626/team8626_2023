package frc.robot.subsystems;
  
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.KitbotDriveTrain;

/* Main class for handling Drivetrain 
**
**
*/

//public class KitbotDriveSubsystem extends SubsystemBase, DriveSubsystem  {
  public class KitbotDriveSubsystem extends DriveSubsystem   {

    private final WPI_VictorSPX m_motorFrontLeft  = new WPI_VictorSPX(KitbotDriveTrain.kCANMotorFL);
    private final WPI_VictorSPX m_motorRearLeft   = new WPI_VictorSPX(KitbotDriveTrain.kCANMotorRL);
    private final WPI_VictorSPX m_motorFrontRight = new WPI_VictorSPX(KitbotDriveTrain.kCANMotorFR);
    private final WPI_VictorSPX m_motorRearRight  = new WPI_VictorSPX(KitbotDriveTrain.kCANMotorRR);

    private final MotorControllerGroup m_motorControllerLeft = new MotorControllerGroup(m_motorFrontLeft, m_motorRearLeft);
    private final MotorControllerGroup m_motorControllerRight = new MotorControllerGroup(m_motorFrontRight, m_motorRearRight);
  
    private final DifferentialDrive m_drive = new DifferentialDrive(m_motorControllerLeft, m_motorControllerRight);
    
    // The left-side drive encoder
    private final Encoder m_leftEncoder =
      new Encoder(
        KitbotDriveTrain.kLeftEncoderPorts[0],
        KitbotDriveTrain.kLeftEncoderPorts[1],
        KitbotDriveTrain.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
      new Encoder(
        KitbotDriveTrain.kRightEncoderPorts[0],
        KitbotDriveTrain.kRightEncoderPorts[1],
        KitbotDriveTrain.kRightEncoderReversed);

    //  Gyro Sensor (Unsing NAVx Module)
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // PID Controllers for Drive Accuracy
    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  
    // Odometry / Kinematics for tracking robot location
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(KitbotDriveTrain.kTrackwidthMeters);
    private final DifferentialDriveOdometry m_odometry;
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(KitbotDriveTrain.ks, KitbotDriveTrain.kv);

    /**
     * Constructs a differential drive object. 
     * Sets the encoder distance per pulse and resets the gyro.
     */
    public KitbotDriveSubsystem() {
      // Reset Gyrometer
      m_gyro.calibrate();
      m_gyro.reset();

      // Set Motor Groups Directions
      m_motorControllerLeft.setInverted(KitbotDriveTrain.kLeftMotorsInverted);
      m_motorControllerRight.setInverted(KitbotDriveTrain.kRightMotorsInverted);

      // Sets distance for per pulse for the encoders
      m_leftEncoder.setDistancePerPulse(KitbotDriveTrain.kEncoderMetersPerPulse);
      m_rightEncoder.setDistancePerPulse(KitbotDriveTrain.kEncoderMetersPerPulse);  

      // Reset Encoders
      resetEncoders();;

      m_odometry =
      new DifferentialDriveOdometry(
          m_gyro.getRotation2d(), 
          m_leftEncoder.getDistance(), 
          m_rightEncoder.getDistance());

      // Set initial Power for the drivetrain
      this.setHighSpeed();

      // Disable motor Safety for Simulation (Avoid Watchdog timeout on motors)
      if(RobotBase.isSimulation()){
        m_drive.setSafetyEnabled(false);
      } 
    }  

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
      final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

      final double leftOutput =
          m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
      final double rightOutput =
          m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
      m_motorControllerLeft.setVoltage(leftOutput + leftFeedforward);
      m_motorControllerRight.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot Angular velocity in rad/s.
     */
    @Override
    public void drive(double xSpeed, double rot) {
      var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
      setSpeeds(wheelSpeeds);
    }

    /** 
     * Updates the field-relative position. 
     * */
    public void updateOdometry() {
      m_odometry.update(
          m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd, rot);
    }

  /**   
   * Drives the robot using tank controls.
   *
   * @param leftSpeed  Left Side Speed
   * @param RightSpeed Right Side Speed
   */
  public void tankDrive(double leftSpeed, double RightSpeed) {
    m_drive.tankDrive(leftSpeed, RightSpeed);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_motorControllerLeft.setVoltage(leftVolts);
    m_motorControllerRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void initDashboard(){
    SmartDashboard.putNumber("Distance Right", this.getEncoderDistanceRight());
    SmartDashboard.putNumber("Distance Left", this.getEncoderDistanceLeft());
    SmartDashboard.putNumber("Heading", this.getHeading());
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Distance Right", this.getEncoderDistanceRight());
    SmartDashboard.putNumber("Distance Left", this.getEncoderDistanceLeft());
    SmartDashboard.putNumber("Heading", this.getHeading());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.updateOdometry();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();

  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

 /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  public double getEncoderDistanceLeft() {
    return m_leftEncoder.getDistance();
  }
  public double getEncoderDistanceRight() {
    return m_rightEncoder.getDistance();
  }

  /** 
   * Resets the drive encoders to currently read a position of 0. 
   **/
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /** 
   * Zeroes the heading of the robot. 
   **/
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  /**
   * Set Low Speed Power for the drivetrain.
   */
  public void setLowSpeed() {
    m_drive.setMaxOutput(KitbotDriveTrain.kPowerRatioLowSpeed);
  }

  /**
   * Set High Speed Power for the drivetrain.
   */
  public void setHighSpeed() {
    m_drive.setMaxOutput(KitbotDriveTrain.kPowerRatioHighSpeed);
  }

  /**
   * Returns Drivetrain Type...
   */
  public boolean isKitBot(){
    return true;
  }
  public boolean isSwerve(){
    return !isKitBot();
  }
}
