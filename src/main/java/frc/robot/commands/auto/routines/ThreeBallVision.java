
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
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.DriveRotation;
import frc.robot.commands.auto.IntakeAuto;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.commands.cargo.Shoot;
import frc.robot.util.BallPositions;
import frc.robot.util.CargoUtil;

public class ThreeBallVision extends SequentialCommandGroup {
  public ThreeBallVision(Alliance color) {
    this(color, Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, Robot.ballDetection);
  }

  public ThreeBallVision(Alliance color, Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight limelight, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, limelight, ballDetection);
    addCommands(
        new InstantCommand(() -> drive.resetOdometry(BallPositions.getBall(3, color).getRobotPoseFromBall())),
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
        new DriveDistance(0.6642),
        new ShootAuto(false, false, 1, () -> true, 157, 25),
        new IntakeAuto(Constants.arm.kAutoBackOuttakeFarPos, false, color == Alliance.Red, 0.4), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new DriveRotation(0.65),
        new InstantCommand(() -> CargoUtil.enableAll()),
        parallel(
          new DriveDistance(1.25),
          new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed))
        ),
        new PositionArm(Constants.arm.kIntakePos),
        new ChaseBall(color == Alliance.Red, false).withTimeout(2),
        new PositionArm(Constants.arm.kFrontOuttakeAutoPos),
        new DriveRotation(-0.6),
        //new ShootAuto(false, true, 0, () -> true, Constants.arm.kFrontOuttakeAutoPos, Constants.shooter.kFrontOuttakeAutoSpeed),
        new Shoot(false, true, false, 154, -3357),
        new PositionArm(154)
    );
  }
}
