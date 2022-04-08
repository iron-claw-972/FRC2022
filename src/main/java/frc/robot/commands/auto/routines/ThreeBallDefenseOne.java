package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.BallPositions;

public class ThreeBallDefenseOne extends SequentialCommandGroup {

  public ThreeBallDefenseOne(Alliance color) {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, color);
  }

  public ThreeBallDefenseOne(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight ll, Alliance color) {
    addRequirements(drive, belt, arm, shooter, ll);
    addCommands(
      new InstantCommand(() -> drive.resetOdometry(BallPositions.B1.getRobotPoseFromBall())),
      new ThreeBall(Alliance.Blue),
      new PathweaverIntake("4balldefenseone", true),
      new ShootAuto(false, true, 0, () -> true, 108, 23)
    );
  }
}
