package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

public class RotatePoseToHub extends SequentialCommandGroup {

  /**
   * 
   * Uses odometry pose estimation to rotate to face the direction of the hub.
   * 
   * @param drive The drivetrain
   */
  public RotatePoseToHub(Drivetrain drive) { 
    Pose2d currentPose = drive.getPose(); // starting position is our current Pose2d
    Pose2d finalPose = new Pose2d(currentPose.getTranslation(), Functions.getAngleToHub(currentPose)); // ending position is our current Translation2d but the Rotation is towards the hub
    Trajectory trajectory = Functions.createTrajectory(currentPose, finalPose); // createTrajectory might not work with only two waypoints
    addCommands(
      new PathweaverCommand(trajectory, drive, false, true)
    );
  }

}
