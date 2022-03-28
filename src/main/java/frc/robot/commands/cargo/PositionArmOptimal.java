package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.ShooterMethods;

public class PositionArmOptimal extends SequentialCommandGroup {
    /**
     * 
     * Enables the shooter arm, and uses pid to go to the specified angle
     * 
     * @param armPosition the pid setpoint for the angle in degrees
     */
    public PositionArmOptimal() {
        addRequirements(Robot.m_arm);
        addCommands(
            new InstantCommand(() -> ShooterMethods.enableArm()),
            new InstantCommand(() -> ShooterMethods.setAngleOptimal()),
            new WaitUntilCommand(() -> ShooterMethods.isArmAtSetpoint())
        );
    }
}
