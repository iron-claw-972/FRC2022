package frc.robot.commands.cargo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.util.CargoUtil;

public class PositionArmOptimal extends SequentialCommandGroup {
    public PositionArmOptimal() {
      this(Robot.arm);
    }

    /**
     * 
     * Enables the shooter arm, and uses pid to go to the specified angle
     * 
     * @param armPosition the pid setpoint for the angle in degrees
     */
    public PositionArmOptimal(Arm arm) {
        addRequirements(arm);
        addCommands(
            new InstantCommand(() -> CargoUtil.enableArm()),
            new InstantCommand(() -> CargoUtil.setAngleOptimal()),
            new WaitUntilCommand(() -> CargoUtil.isArmAtSetpoint())
        );
    }
}
