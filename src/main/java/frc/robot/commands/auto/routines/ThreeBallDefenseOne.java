package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.constants.Constants;
import frc.robot.util.BallPositions;

public class ThreeBallDefenseOne extends SequentialCommandGroup {
    public ThreeBallDefenseOne() {
        addCommands(
            new TwoBallPW(),

            new PathweaverIntake("4balltwo", false, true),
    
            new ShootAuto(false, true, 0, () -> true, 109, 26)

            //TODO: add defense ball
        );
    }
}
