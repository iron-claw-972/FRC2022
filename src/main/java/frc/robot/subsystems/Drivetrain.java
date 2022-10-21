/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_rightMotor1;
  private final WPI_TalonFX m_leftMotor2;
  private final WPI_TalonFX m_rightMotor2;
  
  private final PhoenixMotorControllerGroup m_leftMotors;
  private final PhoenixMotorControllerGroup m_rightMotors;
  public final DifferentialDrive m_dDrive;

  // The left-side drive encoder
  private final TalonEncoder m_leftEncoder;

  // The right-side drive encoder
  private final TalonEncoder m_rightEncoder;

  private final AHRS m_navX;

  public Field2d m_field = new Field2d();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_leftPositionPID = new PIDController(Constants.drive.KpPosition, Constants.drive.KiPosition, Constants.drive.KdPosition);
  private final PIDController m_rightPositionPID = new PIDController(Constants.drive.KpPosition, Constants.drive.KiPosition, Constants.drive.KdPosition);

  private final PIDController m_leftVelocityPID = new PIDController(Constants.drive.KpVelocity,
      Constants.drive.KiVelocity, Constants.drive.KdVelocity);
  private final PIDController m_rightVelocityPID = new PIDController(Constants.drive.KpVelocity,
      Constants.drive.KiVelocity, Constants.drive.KdVelocity);

  private final RamseteController m_ramseteController = new RamseteController(Constants.auto.kRamseteB,
      Constants.auto.kRamseteZeta);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      Constants.drive.KsLinear,
      Constants.drive.KvLinear,
      Constants.drive.KaLinear);

  private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
      Constants.drive.kTrackWidth);

  // These classes help us simulate our drivetrain
  private DifferentialDrivetrainSim m_drivetrainSim;
  private TalonEncoderSim m_leftEncoderSim;
  private TalonEncoderSim m_rightEncoderSim;

  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;

  public Drivetrain() {
    this(
      ControllerFactory.createTalonFX(Constants.drive.leftMotorPorts[0], Constants.drive.kSupplyCurrentLimit, Constants.drive.kSupplyTriggerThreshold, Constants.drive.kSupplyTriggerDuration, Constants.drive.kMainNeutralMode, false),
      ControllerFactory.createTalonFX(Constants.drive.leftMotorPorts[1], Constants.drive.kSupplyCurrentLimit, Constants.drive.kSupplyTriggerThreshold, Constants.drive.kSupplyTriggerDuration, Constants.drive.kNeutralMode, false),
      ControllerFactory.createTalonFX(Constants.drive.rightMotorPorts[0], Constants.drive.kSupplyCurrentLimit, Constants.drive.kSupplyTriggerThreshold, Constants.drive.kSupplyTriggerDuration, Constants.drive.kMainNeutralMode, false),
      ControllerFactory.createTalonFX(Constants.drive.rightMotorPorts[1], Constants.drive.kSupplyCurrentLimit, Constants.drive.kSupplyTriggerThreshold, Constants.drive.kSupplyTriggerDuration, Constants.drive.kNeutralMode, false),
      new AHRS(SPI.Port.kMXP)
    );
  }

  public Drivetrain(WPI_TalonFX leftMotor1, WPI_TalonFX leftMotor2, WPI_TalonFX rightMotor1, WPI_TalonFX rightMotor2, AHRS navX) {
    m_leftMotor1 = leftMotor1;
    m_leftMotor2 = leftMotor2;
    m_rightMotor1 = rightMotor1;
    m_rightMotor2 = rightMotor2;

    // m_leftMotor1.configFactoryDefault();
    // m_leftMotor2.configFactoryDefault();
    // m_rightMotor1.configFactoryDefault();
    // m_rightMotor2.configFactoryDefault();

    m_rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_leftEncoder = new TalonEncoder(m_leftMotor1, Constants.drive.kLeftEncoderReversed);
    m_rightEncoder = new TalonEncoder(m_rightMotor1, Constants.drive.kRightEncoderReversed);

    m_navX = navX;

    m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1);
    m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1);

    // Inverting one side of the drivetrain as to drive forward
    if (RobotBase.isSimulation()) {
      m_leftMotors.setInverted(false);
      m_rightMotors.setInverted(false);
    } else {
      m_leftMotors.setInverted(false);
      m_rightMotors.setInverted(true);
    }


    m_dDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.drive.kDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.drive.kDistancePerPulse);

    resetEncoders();
    zeroHeading();

    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d());

    if (RobotBase.isSimulation()) {
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSim = new DifferentialDrivetrainSim(
          Constants.drive.kDrivetrainPlant,
          Constants.drive.kDriveGearbox,
          Constants.drive.kGearRatio,
          Constants.drive.kTrackWidth,
          Constants.drive.kWheelDiameter / 2.0,
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new TalonEncoderSim(m_leftEncoder);
      m_rightEncoderSim = new TalonEncoderSim(m_rightEncoder);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }

    
  }

  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor1.set(ControlMode.PercentOutput, throttle + turn);
    m_rightMotor1.set(ControlMode.PercentOutput, throttle - turn);
    // m_leftMotor2.set(ControlMode.PercentOutput, throttle + turn);
    // m_rightMotor2.set(ControlMode.PercentOutput, throttle - turn);
    //System.out.println(turn);
    // m_leftMotors.set( throttle + turn );
    // m_rightMotors.set( throttle - turn );

    // m_dDrive.arcadeDrive(throttle, turn);
  }

  public void tankDrive(double left, double right) {
    // m_leftMotor1.set(ControlMode.PercentOutput, left);
    // m_rightMotor1.set(ControlMode.PercentOutput, right);
    // m_leftMotor2.set(ControlMode.PercentOutput, left);
    // m_rightMotor2.set(ControlMode.PercentOutput, right);
    m_dDrive.tankDrive(left, right);
    // m_leftMotors.set(left);
    // m_rightMotors.set(right);
  }

  public void propDrive(double throttle, double turn) {
    double leftOut = throttle * (1 - turn);
    double rightOut = throttle * (1 + turn);

    m_leftMotor1.set(ControlMode.PercentOutput, leftOut);
    m_leftMotor2.set(ControlMode.PercentOutput, leftOut);
    // m_rightMotor1.set(ControlMode.PercentOutput, rightOut);
    // m_rightMotor2.set(ControlMode.PercentOutput, rightOut);
    //m_dDrive.feed();
    //m_dDrive.curvatureDrive(throttle, turn, false);
  }

  public void setBrakeMode() {
    // m_leftMotor1.setNeutralMode(NeutralMode.Brake);
    // m_rightMotor1.setNeutralMode(NeutralMode.Brake);
    // m_leftMotor2.setNeutralMode(NeutralMode.Brake);
    // m_rightMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    // m_leftMotor1.setNeutralMode(NeutralMode.Coast);
    // m_rightMotor1.setNeutralMode(NeutralMode.Coast);
    // m_leftMotor2.setNeutralMode(NeutralMode.Coast);
    // m_rightMotor2.setNeutralMode(NeutralMode.Coast);
  }

  public void setHalfCoast() {
    // m_leftMotor1.setNeutralMode(NeutralMode.Coast);
    // m_rightMotor1.setNeutralMode(NeutralMode.Coast);
    // m_leftMotor2.setNeutralMode(NeutralMode.Brake);
    // m_rightMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void resetCoastBrakeMode() {
    // m_leftMotor1.setNeutralMode(Constants.drive.kMainNeutralMode);
    // m_rightMotor1.setNeutralMode(Constants.drive.kMainNeutralMode);
    // m_leftMotor1.setNeutralMode(Constants.drive.kNeutralMode);
    // m_rightMotor1.setNeutralMode(Constants.drive.kNeutralMode);
  }

  public void shiftDrive(double throttle, double turn) {

    double leftOut = throttle;
    double rightOut = throttle;
    
    if (turn > 0){
      leftOut = leftOut + turn;
    } else if (turn < 0){
      rightOut = rightOut - turn;
    }

    if (leftOut > 1){
      rightOut = rightOut - (leftOut - 1);
      leftOut = 1;
    }
    if (rightOut > 1){
      leftOut = leftOut - (rightOut - 1);
      rightOut = 1;
    }    

    //System.out.println("left: " + leftOut);
    //System.out.println("Right: " + rightOut);

    m_leftMotor1.set(ControlMode.PercentOutput, leftOut);
    m_rightMotor1.set(ControlMode.PercentOutput, rightOut);
    // m_leftMotor2.set(ControlMode.PercentOutput, leftOut);
    // m_rightMotor2.set(ControlMode.PercentOutput, rightOut);
    //m_dDrive.feed();
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSim.setInputs(
        m_leftMotor1.get() * RobotController.getBatteryVoltage(),
        m_rightMotor1.get() * RobotController.getBatteryVoltage());
    m_drivetrainSim.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    // NavX expects clockwise positive, but sim outputs clockwise negative
    angle.set(Math.IEEEremainder(-m_drivetrainSim.getHeading().getDegrees(), 360));
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION
   * ONLY! If you want
   * it to work elsewhere, use the code in
   * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSim.getCurrentDrawAmps();
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return m_driveKinematics;
  }

  
  @Override
  public void periodic() {
    // SmartDashboard.putData("Drivetrain", m_dDrive);
    // Update the odometry in the periodic block
    m_field.setRobotPose(getPose());
    updateOdometry();
    if (RobotBase.isSimulation()) {
      m_fieldSim.setRobotPose(getPose());
    }
  }
  

  public void updateOdometry() {
    m_odometry.update(m_navX.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void feedForwardDrive(double xSpeed, double rot) {
    var wheelSpeeds = m_driveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
    //m_dDrive.feed();
  }

  public void tankFeedForwardDrive(double left, double right) {
    feedForwardDrive(
      (left + right)/2,
      (left - right)/2 
    );
  }

  public RamseteController getRamseteController() {
    return m_ramseteController;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return m_feedforward;
  }

  //Velocity PID for auto
  public PIDController getLeftPositionPID() {
    return m_leftPositionPID;
  }
  public PIDController getRightPositionPID() {
    return m_rightPositionPID;
  }

  //Velocity PID for teleop
  public PIDController getLeftVelocityPID() {
    return m_leftVelocityPID;
  }
  public PIDController getRightVelocityPID() {
    return m_rightVelocityPID;
  }

  public void runDrive(double throttle, double turn) {
   /* switch (Driver.getDriveMode()) {
      case ARCADE:
        arcadeDrive(throttle, turn);
        break;
      case PROPORTIONAL:
        propDrive(throttle, turn);
        break;
      case SHIFT:
        shiftDrive(throttle, turn);
        break;
      default:
        break;
    }*/
    arcadeDrive(throttle, turn);
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
   * Returns the currently-estimated x pose of the robot.
   *
   * @return The x of the pose in meters.
   */
  public double getEstimatedX() {
    return m_odometry.getPoseMeters().getX();
  }

  /**
   * Returns the currently-estimated y pose of the robot.
   *
   * @return The y of the pose in meters.
   */
  public double getEstimatedY() {
    return m_odometry.getPoseMeters().getY();
  }

  /**
   * Returns the currently-estimated rotation pose of the robot.
   *
   * @return The rotation in degrees.
   */
  public double getEstimatedDegrees() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //zeroHeading();
    m_odometry.resetPosition(pose, m_navX.getRotation2d());
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / Constants.kMaxVoltage;
      rightVolts *= batteryVoltage / Constants.kMaxVoltage;
    }
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_dDrive.feed();
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = m_leftVelocityPID.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightVelocityPID.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_dDrive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_navX.getRotation2d().getDegrees();
  }

  public double getPoseX() {
    return getPose().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }

  public double getPoseRotation() {
    return getPose().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_navX.getRate();
  }

  public void updateMotors(){
    m_dDrive.feed();
  }
  
  public double getLeftPosition(){
    return m_leftEncoder.getDistance();
  }

  public double getRightPosition(){
    return m_rightEncoder.getDistance();
  }
}
