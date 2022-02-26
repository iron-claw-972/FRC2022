package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.ShooterMethods;

public class Intake extends SequentialCommandGroup {
  public Intake(double intakeArmPosition, double beltIntakeSpeed, double shooterWheelIntakeSpeed, double postIntakeArmPosition) {
    addCommands(
        new InstantCommand(() -> ShooterMethods.enableAll()),
        new InstantCommand(() -> ShooterMethods.setWheelSpeed(shooterWheelIntakeSpeed)),
        new InstantCommand(() -> ShooterMethods.setBeltPower(beltIntakeSpeed)),
        new InstantCommand(() -> ShooterMethods.setAngle(intakeArmPosition)),
        new WaitUntilCommand(() -> ShooterMethods.isBallContained()),
        new InstantCommand(() -> ShooterMethods.disableShooter()),
        new InstantCommand(() -> ShooterMethods.setAngle(postIntakeArmPosition)),
        new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()),
        new InstantCommand(() -> ShooterMethods.disableBelt()));
  }
}
