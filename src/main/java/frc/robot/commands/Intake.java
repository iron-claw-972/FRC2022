package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMethods;

public class Intake extends SequentialCommandGroup {
  public Intake(
      double intakeArmPosition,
      double beltIntakeSpeed,
      double shooterWheelIntakeSpeed,
      double postIntakeArmPosition,
      boolean doesChaseBall,
      boolean isRedBall) {
    addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt,
        RobotContainer.m_limelight);
    if (doesChaseBall) {
      addRequirements(RobotContainer.m_drive);
    }

    addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        new InstantCommand(() -> ShooterMethods.setWheelSpeed(shooterWheelIntakeSpeed)),
        new InstantCommand(() -> ShooterMethods.setBeltPower(beltIntakeSpeed)),
        new InstantCommand(() -> ShooterMethods.setAngle(intakeArmPosition)),
        (doesChaseBall ? new ChaseBall(RobotContainer.m_limelight, RobotContainer.m_drive, isRedBall)
            : new WaitUntilCommand(() -> ShooterMethods.isBallContained())),
        new InstantCommand(() -> ShooterMethods.disableShiitake()),
        new InstantCommand(() -> RobotContainer.m_cargoRotator.resetPID()),
        new InstantCommand(() -> ShooterMethods.setAngle(postIntakeArmPosition)),
        new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()),
        new InstantCommand(() -> ShooterMethods.disableBelt())
    );
  }
  
  @Override
  public void end(boolean interrupted) {
      ShooterMethods.disableShiitake();
  }
}
