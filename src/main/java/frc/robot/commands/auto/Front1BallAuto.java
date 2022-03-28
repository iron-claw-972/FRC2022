package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.cargo.PositionArm;

public class Front1BallAuto extends SequentialCommandGroup {
  public Front1BallAuto() {
    addRequirements(Robot.m_drive, Robot.m_belt, Robot.m_arm, Robot.m_shooter);
    addCommands(
        new ShootAuto(false, true, 1, () -> true, 108, 22.4719101),
        new DriveDistance(-1.0),
        new PositionArm(154)
    );
  }
}
