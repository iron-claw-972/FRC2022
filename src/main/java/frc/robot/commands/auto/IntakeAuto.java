package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.cargo.ChaseBall;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterMethods;

public class IntakeAuto extends SequentialCommandGroup {
  public IntakeAuto(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall,
      double distance
  ) {
    addRequirements(Robot.m_shooter, Robot.m_arm, Robot.m_belt,
        Robot.m_limelight);
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
          new ChaseBall(Robot.m_limelight, Robot.m_drive, isRedBall),
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
