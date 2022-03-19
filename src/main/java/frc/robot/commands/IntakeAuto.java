package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMethods;

public class IntakeAuto extends SequentialCommandGroup {
  public IntakeAuto(
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall,
      double distance
  ) {
    addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt,
        RobotContainer.m_limelight);
    addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        parallel(
          new InstantCommand(() -> ShooterMethods.setWheelRPM(RobotContainer.wheelConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(RobotContainer.beltConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setAngle(RobotContainer.cargoConstants.kIntakePos))
        ),
        new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()),
        new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint()),
        new DriveDistance(distance),
        new ConditionalCommand(
          new ChaseBall(RobotContainer.m_limelight, RobotContainer.m_drive, isRedBall),
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
