package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.PathweaverCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class TwoBallDefenseOne extends SequentialCommandGroup {

  public TwoBallDefenseOne() {
    this(Robot.drive, Robot.belt, Robot.arm, Robot.shooter, Robot.ll);
  }

  public TwoBallDefenseOne(Drivetrain drive, Belt belt, Arm arm, Shooter shooter, Limelight ll) {
    addRequirements(drive, belt, arm, shooter, ll);
    addCommands(
      new TwoBallPW(),
      new PathweaverCommand("2ball1Defense", false, true)
    );
  }
}
