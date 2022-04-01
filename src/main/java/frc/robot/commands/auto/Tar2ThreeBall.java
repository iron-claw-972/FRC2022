
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.commands.cargo.Shoot;
import frc.robot.util.CargoUtil;

public class Tar2ThreeBall extends SequentialCommandGroup {
  public Tar2ThreeBall() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, Robot.ballDetection);
  }

  public Tar2ThreeBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight limelight, BallDetection ballDetection) {
    addRequirements(drive, belt, arm, shooter, limelight, ballDetection);
    addCommands(
        new InstantCommand(() -> CargoUtil.setBeltPower(0)),
        new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 157, 25),
        parallel(
          new PathweaverCommand("0_pathanthonywantedmetomake", drive),
          new PositionArm(Constants.arm.kIntakePos),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          )
        ),       
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        parallel(
          new PathweaverCommand("1_pathanthonywantedmetomake", drive),
          new PositionArm(Constants.arm.kIntakePos),
          sequence(
            new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
            new InstantCommand(() -> CargoUtil.enableAll())
          )
        ),
        new Shoot(false, true, false, 154, -3357),
        //Line 47 needs values changed to be accurate
        new PositionArm(154)
    );
  }
}
