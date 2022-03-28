package frc.robot.commands.cargo;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.util.ShooterMethods;

public class PositionArm extends SequentialCommandGroup {
    public PositionArm(double armPosition) {
      this(armPosition, Robot.arm);
    }

    /**
     * 
     * Enables the shooter arm, and uses pid to go to the specified angle
     * 
     * @param armPosition the pid setpoint for the angle in degrees
     */
    public PositionArm(double armPosition, Arm arm) {
        addRequirements(arm);
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> ShooterMethods.setAngle(armPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint())
        );
    }
}
