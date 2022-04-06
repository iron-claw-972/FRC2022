package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.util.BallPositions;
import frc.robot.subsystems.Drivetrain;

public class OneBallPW extends SequentialCommandGroup {
  public OneBallPW() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter);
  }

  public OneBallPW(Drivetrain drive, Belt belt, Arm arm, Shooter shooter) {
    addRequirements(drive, belt, arm, shooter);
    addCommands( 
        new InstantCommand(() -> drive.resetOdometry(BallPositions.B3.getRobotPoseFromBall())),
        new ShootAuto(false, true, 1, () -> true, 108, 22.4719101),
        new PathweaverCommand("4ballzero", drive),
        new PathweaverCommand("4ballone", drive),
        new PositionArm(154)
    );
  }
}
