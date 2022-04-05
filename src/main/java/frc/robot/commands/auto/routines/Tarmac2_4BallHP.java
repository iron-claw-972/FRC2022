package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.IntakeAuto;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.BallPositions;
import frc.robot.util.CargoUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class Tarmac2_4BallHP extends SequentialCommandGroup {
  public Tarmac2_4BallHP() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ballDetection);
  }

  public Tarmac2_4BallHP(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, ballDetection);
    addCommands(
        new InstantCommand(() -> drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())),
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 157, 25)
        ),
        new IntakeAuto(Constants.arm.kAutoBackOuttakeFarPos, false, false, Constants.auto.kIntakeDriveDistance), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        parallel(
          new PositionArm(Constants.arm.kIntakePos),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          ),
          new PathweaverCommand("1_pathanthonywantedmetomake", drive)
        ),
        new ShootAuto(false, true, 0, () -> true, 157, 25),
        parallel(
          new PositionArm(Constants.arm.kIntakePos),
          new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
          new PathweaverCommand("2_to4thBall", drive)
        ),
        new PathweaverCommand("3_from4thBall", drive),
        new ShootAuto(false, true, 0, () -> true, 157, 25)
    );
  }
}
