package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.CargoUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class Back2BallAuto extends SequentialCommandGroup {
  public Back2BallAuto() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ballDetection);
  }

  public Back2BallAuto(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, ballDetection);
    addCommands(
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
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
