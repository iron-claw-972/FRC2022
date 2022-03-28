package frc.robot.commands.cargo;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.ShooterMethods;

public class PositionArm extends SequentialCommandGroup {
    public PositionArm(DoubleSupplier armPosition) {
      this(armPosition.getAsDouble());
    }

    /**
     * 
     * Enables the shooter arm, and uses pid to go to the specified angle
     * 
     * @param armPosition the pid setpoint for the angle in degrees
     */
    public PositionArm(double armPosition) {
        addRequirements(Robot.m_arm);
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> ShooterMethods.setAngle(armPosition)),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint())
        );
    }
}
