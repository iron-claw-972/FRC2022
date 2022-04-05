
package frc.robot.commands.auto.routines;

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
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.AlignToUpperHub;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.util.BallPositions;
import frc.robot.util.CargoUtil;

public class Tar2FourBall extends SequentialCommandGroup {
  public Tar2FourBall(Alliance color) {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, Robot.ballDetection, color);
  }

  public Tar2FourBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight limelight, BallDetection ballDetection, Alliance color) {
    addRequirements(drive, belt, arm, shooter, limelight, ballDetection);
    addCommands(
        new InstantCommand(() -> drive.resetOdometry(BallPositions.getBall(3, color).getRobotPoseFromBall())),
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
        new PathweaverCommand("4ballzero", drive),
        new ShootAuto(false, false, 1, () -> true, 157, 24),
        new PositionArm(Constants.arm.kIntakePos),
        new ParallelDeadlineGroup(
          new WaitUntilCommand(() -> CargoUtil.isBallContained()).withTimeout(2),
          new PathweaverCommand("4ballone", drive),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          )
        ),
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        parallel(
          sequence(
            new PathweaverCommand("4balltwo", drive),
            new ChaseBall(false, false)
          ),
          new PositionArm(Constants.arm.kIntakePos),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          )
        ),
        new PositionArm(154),
        new AlignToUpperHub(),
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        parallel(
          sequence(
            new PathweaverCommand("4ballthree", drive),
            new ChaseBall(false, false)
          ),
          new PositionArm(Constants.arm.kIntakePos),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          )
        ),
        new PathweaverCommand("4ballfour", drive),
        new AlignToUpperHub(),
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new PositionArm(154)
    );
  }
}
