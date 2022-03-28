package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;

public class Pathweaver {

  //public static Intake m_intake = Intake.getInstance();
    
  private static Trajectory autonomousTrajectory;
  private static RamseteCommand ramseteCommand;

  public static void setupAutonomousTrajectory(String trajectoryName) {
    setupAutonomousTrajectory(trajectoryName, Robot.drive);
  }

  public static void setupAutonomousTrajectory(String trajectoryName, Drivetrain drive) {

    String trajectoryJSON = "PathWeaver/Paths/" + trajectoryName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      autonomousTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Unable to open trajectory: " + trajectoryJSON + "\n" +
              "Falling back to default trajectory",
          ex.getStackTrace());

      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
          drive.getFeedforward(),
          drive.getDriveKinematics(),
          Constants.kMaxVoltage);

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(
          Constants.auto.kMaxSpeedMetersPerSecond,
          Constants.auto.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(drive.getDriveKinematics())
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);

      // Fallback to default trajectory
      autonomousTrajectory = TrajectoryGenerator.generateTrajectory(
          // Make a square and end where we started
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(3, 0), new Translation2d(3, 3), new Translation2d(0, 3)),
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass config
          config);
    }

    ramseteCommand = new RamseteCommand(
        autonomousTrajectory,
        drive::getPose,
        drive.getRamseteController(),
        drive.getFeedforward(),
        drive.getDriveKinematics(),
        drive::getWheelSpeeds,
        drive.getLeftPositionPID(),
        drive.getRightPositionPID(),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive);

    drive.resetOdometry(autonomousTrajectory.getInitialPose());
  }

  // returns auto command group
  public static Command pathweaverCommand(String path) {
    return pathweaverCommand(path, Robot.drive);
  }
  public static Command pathweaverCommand(String path, Drivetrain drive) {
    // Run path following command, then stop at the end. At the same time intake.
    // "Deadline" is the first command,
    // meaning the whole group will stop once the first command does.
    setupAutonomousTrajectory(path);
    return new SequentialCommandGroup(
      new InstantCommand(() -> drive.setCoastMode()),
      new ParallelDeadlineGroup(
        ramseteCommand.andThen(new InstantCommand(() -> drive.tankDriveVolts(0, 0)))));
        // new RunCommand(() -> Drivetrain.getInstance().tankDrive(0.5, -0.5), Drivetrain.getInstance()));
  }
}
