package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.IntakeAuto;
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

public class TwoBall extends SequentialCommandGroup {
  public TwoBall() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ballDetection);
  }

  public TwoBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, ballDetection);
    addCommands(
        new InstantCommand(() -> drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())),
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
        new DriveDistance(0.6642),
        new ShootAuto(false, false, 1, () -> true, 157, 25),
        new PrintCommand("finished shooting"),
        new IntakeAuto(Constants.arm.kAutoBackOuttakeFarPos, false, false, 0.4), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new PositionArm(154),
        new DriveDistance(0.1)
    );
  }
}
