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
            
            new InstantCommand(() -> Robot.drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())),

            new PathweaverCommand("4balldefenseone", false, true),
            new PathweaverIntake("4", true),
            new ShootAuto(false, true, 0, () -> false, 153, 24)
        );
    }
}
