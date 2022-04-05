package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Robot;
import frc.robot.constants.Constants;

public class Functions {

  /*
  public static Functions controls;
  public static Functions getControls(){
    return controls;
  }
  */

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
   * linear from (deadband, 0) to (1,1)
   * 
   * @param input    The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadband(double deadband, double input) {
    if (Math.abs(input) <= deadband) {
        return 0;
    } else if (Math.abs(input) == 1) {
        return input;
    } else {
        return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

  //is an exponential function that maintains positive or negative
  public static double expoMS(double exponent, double base) {
    //weird stuff will happen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base),exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }

  // TODO: figure out if this can work?
  public static Trajectory createTrajectory(List<Pose2d> waypoints) {

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        Robot.drive.getFeedforward(),
        Robot.drive.getDriveKinematics(),
        Constants.kMaxVoltage);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.auto.kMaxSpeedMetersPerSecond,
        Constants.auto.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Robot.drive.getDriveKinematics())
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // Fallback to default trajectory
    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  public static Trajectory centerToRobot(Trajectory inputTrajectory) {
    Pose2d bOrigin = new Pose2d(Constants.drive.kRobotWidth/2.0, -Constants.drive.kRobotLength/2.0, Rotation2d.fromDegrees(0));
    return inputTrajectory.relativeTo(bOrigin);
  }

  public static Pose2d centerToRobot(Pose2d inputPose) {
    Pose2d bOrigin = new Pose2d(Constants.drive.kRobotWidth/2.0, -Constants.drive.kRobotLength/2.0, Rotation2d.fromDegrees(0));
    return inputPose.relativeTo(bOrigin);
  }
}
