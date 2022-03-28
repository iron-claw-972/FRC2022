package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterMethods;

public class Back2BallAuto extends SequentialCommandGroup {
  public Back2BallAuto() {
    addRequirements(Robot.m_drive, Robot.m_belt, Robot.m_arm, Robot.m_shooter);
    addCommands(
        new InstantCommand(() -> ShooterMethods.setBeltPower(0)),
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 157, 25)
        ),
        new IntakeAuto(Constants.arm.kAutoBackOuttakeFarPos, false, false, Constants.auto.kIntakeDriveDistance), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new PositionArm(154),
        new DriveDistance(0.1)
    );
  }
}
