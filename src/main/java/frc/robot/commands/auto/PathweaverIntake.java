package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.cargo.PositionArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.util.CargoUtil;

public class PathweaverIntake extends SequentialCommandGroup {

  /**
   * 
   * Runs pathweaver while also setting the arm down and starting to intake.
   * 
   * @param trajectory A string of the trajectory path without anything after the dot
   */
  public PathweaverIntake(String trajectory) {
    this(trajectory, Robot.drive, Robot.arm, Robot.shooter, Robot.belt);
  }

  /**
   * 
   * Runs pathweaver while also setting the arm down and starting to intake.
   * 
   * @param trajectory A string of the trajectory path without anything after the dot
   * @param drive
   * @param arm
   * @param shooter
   * @param belt
   */
  public PathweaverIntake(String trajectory, Drivetrain drive, Arm arm, Shooter shooter, Belt belt) {
    addRequirements(drive, arm, shooter, belt);
    addCommands(
      parallel(
        new PathweaverCommand(trajectory, drive),
        new PositionArm(Constants.arm.kIntakePos),
        sequence(
          new InstantCommand(() -> CargoUtil.setWheelRPM(Constants.shooter.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.setBeltPower(Constants.belt.kIntakeSpeed)),
          new InstantCommand(() -> CargoUtil.enableAll())
        )
      )
    );
  }
  
}
