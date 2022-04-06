package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;

import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

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
        "Unable to open trajectory: " + trajectoryName + "\n" + "Will not do anything.",
        ex.getStackTrace()
      );
      return null;
    }
  }

  public PathweaverCommand(String trajectoryName, Drivetrain drive) {
    this(
      getTrajectory(trajectoryName, drive),
      drive,
      false
    );
  }

  public PathweaverCommand(String trajectoryName, Drivetrain drive, boolean resetPose) {
    this(
      getTrajectory(trajectoryName, drive),
      drive,
      resetPose
    );
  }

  public PathweaverCommand(Trajectory trajectory, Drivetrain drive) {
    this(trajectory, drive, false);
  }

  public PathweaverCommand(Trajectory trajectory, Drivetrain drive, boolean resetPose) {
    m_drive = drive;
    addRequirements(drive);

    if (trajectory != null) {

      Trajectory newTrajectory = Functions.centerToRobot(trajectory);

      addCommands(
        (resetPose ? new InstantCommand(() -> drive.resetOdometry(newTrajectory.getInitialPose())) : new DoNothing()),
        new InstantCommand(() -> drive.m_field.getObject("traj").setTrajectory(trajectory)),
        new RamseteCommand(
          newTrajectory,
          drive::getPose,
          drive.getRamseteController(),
          drive.getFeedforward(),
          drive.getDriveKinematics(),
          drive::getWheelSpeeds,
          drive.getLeftVelocityPID(),
          drive.getRightVelocityPID(),
          drive::tankDriveVolts,
          drive
        ),
        new InstantCommand(() -> m_drive.tankDriveVolts(0, 0))
      );
    } else {
      addCommands(new DoNothing());
    }
  }
}
