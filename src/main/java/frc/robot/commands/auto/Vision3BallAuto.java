
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.commands.cargo.Shoot;
import frc.robot.util.ShooterMethods;

public class Vision3BallAuto extends SequentialCommandGroup {
  public Vision3BallAuto(boolean isRedBall) {
    addRequirements(Robot.m_drive, Robot.m_belt, Robot.m_arm, Robot.m_shooter);
    addCommands(
        new InstantCommand(() -> ShooterMethods.setBeltPower(0)),
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 157, 25)
        ),
        new IntakeAuto(Constants.arm.kAutoBackOuttakeFarPos, false, isRedBall, Constants.auto.kIntakeDriveDistance), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new DriveRotation(0.65),
        new InstantCommand(() -> ShooterMethods.enableAll()),
        parallel(
          new DriveDistance(1.25),
          new InstantCommand(() -> ShooterMethods.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kIntakeSpeed))
        ),
        new PositionArm(Constants.arm.kIntakePos),
        new ChaseBall(Robot.m_limelight, Robot.m_drive, isRedBall, false).withTimeout(2),
        new PositionArm(Constants.arm.kFrontOuttakeAutoPos),
        new DriveRotation(-0.6),
        //new ShootAuto(false, true, 0, () -> true, Constants.arm.kFrontOuttakeAutoPos, Constants.shooter.kFrontOuttakeAutoSpeed),
        new Shoot(false, true, false, 154, -3357),
        new PositionArm(154)
    );
  }
}
