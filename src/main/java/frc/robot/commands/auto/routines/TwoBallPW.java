package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.util.CargoUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class TwoBallPW extends SequentialCommandGroup {
  public TwoBallPW() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ballDetection);
  }

  public TwoBallPW(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, ballDetection);
    addCommands(
      new InstantCommand(() -> CargoUtil.setBeltPower(0)),
      new PathweaverCommand("4ballzero", true, true),
      new ShootAuto(false, false, 0, () -> true, 157, 24.1),

      new PositionArm(Constants.arm.kIntakePos), //position arm early because it tends to hit the ball
      new PathweaverIntake("4ballone", true),

      new ShootAuto(false, false, 0, () -> true, 157, 25.5)
    );
  }
}
