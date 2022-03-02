package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMethods;

public class Shoot extends SequentialCommandGroup {
    public Shoot(double outtakeArmPosition, double beltIntakeSpeed, double shooterWheelOuttakeSpeed, double beltOuttakeSpeed, boolean doesAlign) {
        if (doesAlign) {
            addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt, RobotContainer.m_limelight, RobotContainer.m_drive);
        } else {
            addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt, RobotContainer.m_limelight);
        }
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> RobotContainer.m_cargoRotator.resetPID()),
            new InstantCommand(() -> ShooterMethods.setAngle(outtakeArmPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1),
            (doesAlign ? new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive).withTimeout(2) : new DoNothing()),
            new InstantCommand(() -> ShooterMethods.enableWheel()),
            new InstantCommand(() -> ShooterMethods.enableBelt()),
            new InstantCommand(() -> ShooterMethods.setBeltPower(beltIntakeSpeed)),
            new InstantCommand(() -> ShooterMethods.setWheelSpeed(shooterWheelOuttakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint()),
            new InstantCommand(() -> ShooterMethods.setBeltPower(beltOuttakeSpeed)),
            // new WaitUntilCommand(() -> ShooterMethods.isBallShot()),
            new WaitCommand(0.5)
        );
    }

    @Override
    public void end(boolean interrupted) {
        ShooterMethods.disableShiitake();
    }
}
