
package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.commands.auto.PathweaverIntake;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.util.BallPositions;
import frc.robot.util.CargoUtil;

public class ThreeBall extends SequentialCommandGroup {
  public ThreeBall(Alliance color) {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll, Robot.ballDetection, color);
  }

  public ThreeBall(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight limelight, BallDetection ballDetection, Alliance color) {
    addRequirements(drive, belt, arm, shooter, limelight, ballDetection);
    addCommands(
        new TwoBallPW(),

        new PathweaverIntake("4balltwo"),

        new ShootAuto(false, true, 0, () -> true, 109, 26)
    );
  }
}
