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

public class TwoBallTest extends SequentialCommandGroup {
  public TwoBallTest() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ballDetection);
  }

  public TwoBallTest(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, ballDetection);
    addCommands(
      new InstantCommand(() -> CargoUtil.setBeltPower(0)),
      parallel(
        sequence(
          new InstantCommand(() -> CargoUtil.setBeltSpeed(Constants.belt.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.enableBelt()),
          new InstantCommand(() -> CargoUtil.setWheelSpeed(() -> 24.0, false)),
          new InstantCommand(() -> CargoUtil.enableWheel())
        ),
        new PathweaverCommand("4ballzero", true, true)
      ),
      new ShootAuto(false, false, 0, () -> true, 157, 24.15),

      new PositionArm(Constants.arm.kIntakePos), //position arm early because it tends to hit the ball
      new PathweaverIntake("4ballone", true),

      new ShootAuto(false, false, 0, () -> true, 157, 25.5),

      new PathweaverCommand("extraforward", false, true)
    );
  }
}
