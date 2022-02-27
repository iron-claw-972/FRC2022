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
        addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt, RobotContainer.m_limelight);
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> ShooterMethods.setAngle(outtakeArmPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()),
            new ParallelCommandGroup(
                (doesAlign ? new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive) : new DoNothing()),
                new SequentialCommandGroup(
                    new InstantCommand(() -> ShooterMethods.enableWheel()),
                    new InstantCommand(() -> ShooterMethods.setBeltSpeed(beltIntakeSpeed)),
                    new InstantCommand(() -> ShooterMethods.setWheelSpeed(ShooterMethods.getOptimalShooterSpeed())),
                    new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint())
                )
            ),
            new InstantCommand(() -> ShooterMethods.setBeltSpeed(beltOuttakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isBallShot()),
            new WaitCommand(0.25),
            new InstantCommand(() -> ShooterMethods.disableShiitake()),
            new InstantCommand(() -> ShooterMethods.disableBelt())
        );
    }
}
