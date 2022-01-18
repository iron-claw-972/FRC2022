/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ControllerFactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Drivetrain extends SubsystemBase {
  
  WPI_TalonFX leftMotor = ControllerFactory.createTalonFX(DriveConstants.kLeftMotorPort);
  //WPI_TalonFX leftMotorPal = ControllerFactory.createTalonFX(DriveConstants.kLeftMotorPalPort);

  WPI_TalonFX rightMotor = ControllerFactory.createTalonFX(DriveConstants.kRightMotorPort);
  //WPI_TalonFX rightMotorPal = ControllerFactory.createTalonFX(DriveConstants.kRightMotorPalPort);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftMotor/*, leftMotorPal*/);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightMotor/*, rightMotorPal*/);

   // The robot's drive
   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
 
   // The gyro sensor
   private final Gyro m_gyro = new ADXRS450_Gyro();
 
   // Odometry class for tracking robot pose
   private final DifferentialDriveOdometry m_odometry;

  public Drivetrain() {
    // leftMotorPal.follow(leftMotor);
    // rightMotorPal.follow(rightMotor);

    // Inverting opposite sides of the drivetrain
    m_rightMotors.setInverted(true);
    
    setEncoders(0, 0);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  int sensitivity = 5;
  public void modSensitivity(){
    if (sensitivity == 5) {
      sensitivity = 2;
      System.out.println("sensitivity changed to 1/2");
    } else {
      sensitivity = 5;
      System.out.println("sensitivity changed to 1/5");
    }
  }

  public void arcadeDrive(double throttle, double turn) {
    leftMotor.set(ControlMode.PercentOutput, (throttle + turn) / sensitivity);
    rightMotor.set(ControlMode.PercentOutput, (throttle - turn) / sensitivity);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), leftMotor.getSelectedSensorPosition(), rightMotor.getSelectedSensorPosition());
  }

  public void setEncoders(double left, double right) {
    leftMotor.setSelectedSensorPosition(left);
    rightMotor.setSelectedSensorPosition(right);
  } 

  public double getLeftEncoder() {
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightEncoder() {
    return rightMotor.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return leftMotor.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightMotor.getSelectedSensorVelocity();
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
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    setEncoders(0, 0);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
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
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoder() + getRightEncoder()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
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

}
