package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.ShooterMethods;

public class PositionArm extends SequentialCommandGroup {
    public PositionArm(double armPosition) {
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> ShooterMethods.setAngle(armPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint())
        );
    }
}
