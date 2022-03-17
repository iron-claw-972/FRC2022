package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimbExtender;

public class ZeroExtender extends SequentialCommandGroup{
    
    ClimbExtender extender;

    public ZeroExtender(ClimbExtender extender){
        addRequirements(extender);
        this.extender = extender;
        addCommands(
            new InstantCommand(() -> extender.disable()), //disable just to make sure PID doesn't run
            new InstantCommand(() -> extender.setOutput(-0.2)),
            new WaitCommand(0.2),
            new WaitUntilCommand(this::compressed),
            new InstantCommand(() -> extender.disable()),
            new InstantCommand(() -> extender.zero())

        );
    }

    private boolean compressed() {
        return extender.compressionLimitSwitch() || Math.abs(extender.getVelocity()) < 0.01;
    }
}
