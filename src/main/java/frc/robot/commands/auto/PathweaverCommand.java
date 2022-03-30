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

public class PathweaverCommand extends SequentialCommandGroup {
  private Drivetrain m_drive;

  private static Trajectory getTrajectory(String trajectoryName, Drivetrain drive) {
    try {
      return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(
          Constants.auto.kTrajectoryDirectory + trajectoryName + ".wpilib.json"
        )
      );
    } catch (IOException ex) {
      DriverStation.reportWarning(
        "Unable to open trajectory: " + trajectoryName + "\n" + "Falling back to default trajectory",
        ex.getStackTrace()
      );

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
      return TrajectoryGenerator.generateTrajectory(
          // Make a square and end where we started
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(3, 0)),
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass config
          config);
    }
  }

  public PathweaverCommand(String trajectoryName, Drivetrain drive) {
    this(
      getTrajectory(trajectoryName, drive),
      drive
    );
  }

  public PathweaverCommand(Trajectory trajectory, Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);

    addCommands(
      new InstantCommand(() -> m_drive.resetOdometry(trajectory.getInitialPose())),
      new RamseteCommand(
        trajectory,
        drive::getPose,
        drive.getRamseteController(),
        drive.getFeedforward(),
        drive.getDriveKinematics(),
        drive::getWheelSpeeds,
        drive.getLeftPositionPID(),
        drive.getRightPositionPID(),
        drive::tankDriveVolts,
        drive
      ),
      new InstantCommand(() -> m_drive.tankDriveVolts(0, 0))
    );
  }
}
