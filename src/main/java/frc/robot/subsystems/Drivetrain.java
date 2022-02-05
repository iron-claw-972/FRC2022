/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.drivetrain.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

import ctre_shims.PhoenixMotorControllerGroup;
import ctre_shims.TalonEncoder;
import ctre_shims.TalonEncoderSim;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.controls.Driver;
import frc.robot.util.ControllerFactory;

public class Drivetrain extends SubsystemBase {

  //change this to use constants from a different robot
  public static ClassBot3Constants kDrivetrain = new ClassBot3Constants();

  private static Drivetrain instance;

  WPI_TalonFX m_leftMotor1 = ControllerFactory.createTalonFX(kDrivetrain.leftMotorPorts[0]);
  WPI_TalonFX m_rightMotor1 = ControllerFactory.createTalonFX(kDrivetrain.rightMotorPorts[0]);
  private PhoenixMotorControllerGroup m_leftMotors;
  private PhoenixMotorControllerGroup m_rightMotors;
  private final DifferentialDrive m_dDrive;

  // The left-side drive encoder
  private final TalonEncoder m_leftEncoder = new TalonEncoder(m_leftMotor1, kDrivetrain.kLeftEncoderReversed);

  // The right-side drive encoder
  private final TalonEncoder m_rightEncoder = new TalonEncoder(m_rightMotor1, kDrivetrain.kRightEncoderReversed);

  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_leftRamsetePIDController = new PIDController(kDrivetrain.kRamseteP, 0, 0);
  private final PIDController m_rightRamsetePIDController = new PIDController(kDrivetrain.kRamseteP, 0, 0);

  private final PIDController m_leftVelocityPIDController = new PIDController(kDrivetrain.kVelocityP,
      kDrivetrain.kVelocityI, kDrivetrain.kVelocityD);
  private final PIDController m_rightVelocityPIDController = new PIDController(kDrivetrain.kVelocityP,
      kDrivetrain.kVelocityI, kDrivetrain.kVelocityD);

  private final RamseteController m_ramseteController = new RamseteController(AutoConstants.kRamseteB,
      AutoConstants.kRamseteZeta);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(
      kDrivetrain.ksVolts,
      kDrivetrain.kvVoltSecondsPerMeter,
      kDrivetrain.kaVoltSecondsSquaredPerMeter);

  private final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(
      kDrivetrain.kTrackWidthMeters);

/*
  // These classes help us simulate our drivetrain
  private DifferentialDrivetrainSim m_drivetrainSim;
  private TalonEncoderSim m_leftEncoderSim;
  private TalonEncoderSim m_rightEncoderSim;

  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim;
*/

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
      return instance;
    }
    return instance;
  }

  public Drivetrain() {

    // go through non main motors and put them in an array (allows for variable # of motors)
    // for loop starts at one because the main motor of that side is already accounted for

    MotorController[] lMotors = new MotorController[kDrivetrain.leftMotorPorts.length];
    for (int i = 1; i < kDrivetrain.leftMotorPorts.length; i++) {
      lMotors[i] = ControllerFactory.createTalonFX(kDrivetrain.leftMotorPorts[i]);
    }

    MotorController[] rMotors = new MotorController[kDrivetrain.rightMotorPorts.length];
    for (int i = 1; i < kDrivetrain.rightMotorPorts.length; i++) {
      rMotors[i] = ControllerFactory.createTalonFX(kDrivetrain.rightMotorPorts[i]);
    }

    if (kDrivetrain.leftMotorPorts.length > 1) {
      m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1, lMotors);
    } else {
      m_leftMotors = new PhoenixMotorControllerGroup(m_leftMotor1);
    }

    if (kDrivetrain.rightMotorPorts.length > 1) {
      m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1, rMotors);
    } else {
      m_rightMotors = new PhoenixMotorControllerGroup(m_rightMotor1);
    }

    m_dDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // Inverting one side of the drivetrain as to drive forward
    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(false);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(kDrivetrain.kEncoderMetersPerPulse);
    m_rightEncoder.setDistancePerPulse(kDrivetrain.kEncoderMetersPerPulse);

    resetEncoders();
    zeroHeading();

    m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d());
    /*
    if (RobotBase.isSimulation()) {
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSim = new DifferentialDrivetrainSim(
          drivetrain.kDrivetrainPlant,
          drivetrain.kDriveGearbox,
          drivetrain.kGearRatio,
          drivetrain.kTrackWidthMeters,
          drivetrain.kWheelDiameterMeters / 2.0,
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new TalonEncoderSim(m_leftEncoder);
      m_rightEncoderSim = new TalonEncoderSim(m_rightEncoder);

      // the Field2d class lets us visualize our robot in the simulation GUI.
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    }
    */
  }

  public void arcadeDrive(double throttle, double turn) {
    // System.out.println("arcade drive");
    m_dDrive.arcadeDrive(throttle, turn);
  }

  public void tankDrive(double left, double right) {
    m_dDrive.tankDrive(left, right);
  }

  public void propDrive(double throttle, double turn){
    double leftOut = throttle * (1 + turn);
    double rightOut = throttle * (1 - turn);

    m_leftMotor1.set(ControlMode.PercentOutput, leftOut);
    m_rightMotor1.set(ControlMode.PercentOutput, rightOut);
    m_dDrive.feed();
  }

  public void shiftDrive(double throttle, double turn) {

    System.out.println("throttle: " + throttle);
    System.out.println("turn: " + turn);

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

    System.out.println("left: " + leftOut);
    System.out.println("Right: " + rightOut);

    m_leftMotor1.set(ControlMode.PercentOutput, leftOut);
    m_rightMotor1.set(ControlMode.PercentOutput, rightOut);
    m_dDrive.feed();
  }

  //sim stuff (need more detail)
  /*
  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSim.setInputs(
        m_leftMotors.get() * RobotController.getBatteryVoltage(),
        m_rightMotors.get() * RobotController.getBatteryVoltage());
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
  */

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION
   * ONLY! If you want
   * it to work elsewhere, use the code in
   * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  /*
   public double getDrawnCurrentAmps() {
    return m_drivetrainSim.getCurrentDrawAmps();
  }
  */

  public DifferentialDriveKinematics getDriveKinematics() {
    return m_driveKinematics;
  }

  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    // if (RobotBase.isSimulation()) {
    //   m_fieldSim.setRobotPose(getPose());
    // }
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
  }

  public void tankFeedForwardDrive(double left, double right){
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
  public PIDController getLeftRamsetePIDController() {
    return m_leftRamsetePIDController;
  }
  public PIDController getRightRamsetePIDController() {
    return m_rightRamsetePIDController;
  }

  //Velocity PID for teleop
  public PIDController getLeftVelocityPIDController() {
    return m_leftVelocityPIDController;
  }
  public PIDController getRightVelocityPIDController() {
    return m_rightVelocityPIDController;
  }

  public void runDrive(double throttle, double turn){
    switch (Driver.getDriveMode()) {
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
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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

    final double leftOutput = m_leftVelocityPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightVelocityPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

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

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_navX.getRate();
  }
}
