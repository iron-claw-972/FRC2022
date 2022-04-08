package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.CargoUtil;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class IntakeAuto extends SequentialCommandGroup {
  public IntakeAuto(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall,
      double distance
  ) {
    this(
      postIntakeArmPosition,
      doesChaseBall,
      isRedBall,
      distance,
      Robot.ll,
      Robot.belt,
      Robot.arm,
      Robot.shooter,
      Robot.drive,
      Robot.ballDetection
    );
  }

  public IntakeAuto(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall,
      double distance,
      Limelight limelight,
      Belt belt,
      Arm arm,
      Shooter shooter,
      Drivetrain drive,
      BallDetection ballDetection
  ) {
    addRequirements(shooter, arm, belt,
        limelight, drive, ballDetection);
    addCommands(
        new InstantCommand(() -> CargoUtil.enableAll()),
        parallel(
          new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
          new PositionArm(Constants.arm.kIntakePos)
        ),
        new WaitUntilCommand(() -> CargoUtil.isWheelAtSetpoint()),
        new PrintCommand("Starting drive distance.. " + distance ),
        new DriveDistance(distance),
        new ConditionalCommand(
          new ChaseBall(isRedBall, false),
          new DoNothing(),
          () -> doesChaseBall
        ),
        new InstantCommand(() -> CargoUtil.disableShiitake()),
        new InstantCommand(() -> CargoUtil.setAngle(postIntakeArmPosition)),
        new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint()).withTimeout(1)
    );
  }

  @Override
  public void end(boolean interrupted) {
    CargoUtil.disableShiitake();
  }
}
