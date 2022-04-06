package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;

public class OneBall extends SequentialCommandGroup {
  public OneBall() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter);
  }

  public OneBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter) {
    addRequirements(drive, belt, arm, shooter);
    addCommands(
        new ShootAuto(false, true, 1, () -> true, 108, 22.4719101),
        new DriveDistance(-1.0),
        new PositionArm(154)
    );
  }
}
