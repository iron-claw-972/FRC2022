package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ShooterMethods;

public class Shoot extends SequentialCommandGroup {
    public Shoot(
          double outtakeArmPosition,
          double beltIntakeSpeed,
          double shooterWheelOuttakeSpeed,
          double beltOuttakeSpeed,
          boolean doesAlign,
          double waitTime) {
        addRequirements(RobotContainer.m_cargoShooter, RobotContainer.m_cargoRotator, RobotContainer.m_cargoBelt, RobotContainer.m_limelight);
        if (doesAlign) {
            addRequirements(RobotContainer.m_drive);
        }
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableAll()),
            new ParallelCommandGroup(
                new WaitCommand(waitTime),
                new SequentialCommandGroup(
                    new InstantCommand(() -> ShooterMethods.setBeltPower(beltIntakeSpeed)),
                    new WaitUntilCommand(() -> ShooterMethods.isBallContained()).withTimeout(1),
                    new InstantCommand(() -> ShooterMethods.setWheelSpeed(SmartDashboard.getNumber("Test shooter velocity", 0))),
                    new WaitUntilCommand(() -> ShooterMethods.isWheelAtSetpoint())
                ),
                new SequentialCommandGroup(
                    new InstantCommand(() -> ShooterMethods.setAngle(SmartDashboard.getNumber("Test arm angle", 0))),
                    new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint()).withTimeout(1)
                ),
                (doesAlign ? new AlignToUpperHub(RobotContainer.m_limelight, RobotContainer.m_drive) : new DoNothing()).withTimeout(2)
            ).withTimeout(4),
            new InstantCommand(() -> ShooterMethods.setBeltPower(beltOuttakeSpeed)),
            new WaitUntilCommand(() -> ShooterMethods.isBallShot()).withTimeout(1)
        );
    }

    @Override
    public void end(boolean interrupted) {
        ShooterMethods.disableShiitake();
    }
}
