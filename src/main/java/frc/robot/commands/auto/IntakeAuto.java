package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;
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
      Robot.limelight,
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
        new InstantCommand(() -> ShooterMethods.enableAll()),
        parallel(
          new InstantCommand(() -> ShooterMethods.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(Constants.belt.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setAngle(Constants.arm.kIntakePos))
        ),
        new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()),
        new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint()),
        new DriveDistance(distance),
        new ConditionalCommand(
          new ChaseBall(isRedBall, false),
          new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(1.5),
          () -> doesChaseBall
        ),
        new InstantCommand(() -> ShooterMethods.disableShiitake()),
        new InstantCommand(() -> ShooterMethods.setAngle(postIntakeArmPosition)),
        new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
    );
  }

  @Override
  public void end(boolean interrupted) {
    ShooterMethods.disableShiitake();
  }
}
