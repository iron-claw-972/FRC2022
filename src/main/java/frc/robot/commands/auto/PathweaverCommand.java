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

  public PathweaverCommand(String trajectoryName, Drivetrain drive) {
    this(
      Functions.getTrajectory(trajectoryName),
      drive,
      false
    );
  }

  public PathweaverCommand(String trajectoryName, Drivetrain drive, boolean resetPose) {
    this(
      Functions.getTrajectory(trajectoryName),
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

      addCommands(
        (resetPose ? new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())) : new DoNothing()),
        new InstantCommand(() -> drive.m_field.getObject("traj").setTrajectory(trajectory)),
        new RamseteCommand(
          trajectory,
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
