package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

public class PathweaverCommand extends SequentialCommandGroup {
  private Drivetrain m_drive;

  /**
   * 
   * Creates a command that runs pathweaver path from a pathweaver generated file.
   * 
   * @param trajectoryName The trajectory for pathweaver to run. This is the file name in deploy/paths/output but without the stuff after the dot.
   * @param resetPose Whether or not to reset the pose before starting.
   * @param stopAtEnd Whether or not to set the motor power to zero at the end of the command.
   */
  public PathweaverCommand(String trajectoryName, boolean resetPose, boolean stopAtEnd) {
    this(
      Functions.getTrajectory(trajectoryName),
      Robot.drive,
      resetPose, 
      stopAtEnd
    );
  }

  /**
   * 
   * Creates a command that runs pathweaver path from a pathweaver generated file.
   * 
   * @param trajectoryName The trajectory for pathweaver to run. This is the file name in deploy/paths/output but without the stuff after the dot.
   * @param drive The robot drivetrain.
   * @param resetPose Whether or not to reset the pose before starting.
   * @param stopAtEnd Whether or not to set the motor power to zero at the end of the command.
   */
  public PathweaverCommand(String trajectoryName, Drivetrain drive, boolean resetPose, boolean stopAtEnd) {
    this(
      Functions.getTrajectory(trajectoryName),
      drive,
      resetPose, 
      stopAtEnd
    );
  }

  /**
   * 
   * Creates a command that runs pathweaver.
   * 
   * @param trajectory The trajectory for pathweaver to run.
   * @param drive The robot drivetrain.
   * @param resetPose Whether or not to reset the pose before starting.
   * @param stopAtEnd Whether or not to set the motor power to zero at the end of the command.
   */
  public PathweaverCommand(Trajectory trajectory, Drivetrain drive, boolean resetPose, boolean stopAtEnd) {
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
        (stopAtEnd ? new InstantCommand(() -> m_drive.tankDriveVolts(0, 0)) : new DoNothing())
      );
    } else {
      addCommands(new DoNothing());
    }
  }
}
