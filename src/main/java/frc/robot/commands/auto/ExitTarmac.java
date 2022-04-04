
package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.cargo.AlignToUpperHub;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.util.BallPositions;
import frc.robot.util.CargoUtil;
import frc.robot.util.Functions;

public class ExitTarmac extends SequentialCommandGroup {
  public ExitTarmac(Alliance color) {
    this(Robot.drive, color);
  }

  public ExitTarmac(Drivetrain drive, Alliance color) {
    addRequirements(drive);
    addCommands(
      new InstantCommand(() -> drive.resetOdometry(BallPositions.getBall(3, color).getRobotPoseFromBall())),
      new PathweaverCommand(
        Functions.createTrajectory(
          List.of(
            BallPositions.getBall(3, color).getRobotPoseFromBall(),
            new Pose2d(BallPositions.getBall(3, color).m_pos, new Rotation2d(BallPositions.getBall(3, color).m_angleAwayFromHub)))), 
        drive)
    );
  }
}
