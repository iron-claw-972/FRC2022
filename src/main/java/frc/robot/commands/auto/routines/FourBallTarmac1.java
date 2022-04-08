package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.BallPositions;

public class FourBallTarmac1 extends SequentialCommandGroup {

  public FourBallTarmac1(Alliance color) {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, color);
  }

  public FourBallTarmac1(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight ll, Alliance color) {
      
    addRequirements(drive, belt, arm, shooter, ll);
    addCommands(
      new InstantCommand(() -> drive.resetOdometry(BallPositions.B1.getRobotPoseFromBall())),

      new ShootAuto(false, true, 1, () -> true, 108, 22.4719101),

      new PathweaverIntake("3BallTurn_0", false, true),
      new ShootAuto(false, true, 0, () -> true, 157, 25),
      new PathweaverCommand("3BallTurn_1", false, false),
      new PathweaverIntake("3BallTurn_2", false, true),
      new ShootAuto(false, true, 0, () -> true, 153, 25),
      new PathweaverIntake("3BallTurn_3", false, false),
      new PathweaverCommand("4ballfour", false, true),
      new ShootAuto(false, true, 0, () -> true, 108, 32)
    );
  }

}