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
   * Runs pathweaver while also setting the arm down and starting to intake. Will not reset the pose before running.
   * 
   * @param trajectory A string of the trajectory path without anything after the dot.
   * @param stopAtEnd Whether or not to stop the motors at the end of the path.
   */
  public PathweaverIntake(String trajectory, boolean stopAtEnd) {
    this(trajectory, Robot.drive, Robot.arm, Robot.shooter, Robot.belt, false, stopAtEnd);
  }

  /**
   * 
   * Runs pathweaver while also setting the arm down and starting to intake.
   * 
   * @param trajectory A string of the trajectory path without anything after the dot.
   * @param resetPose Whether or not to reset the pose before starting the path.
   * @param stopAtEnd Whether or not to stop the motors at the end of the path.
   */
  public PathweaverIntake(String trajectory, boolean resetPose, boolean stopAtEnd) {
    this(trajectory, Robot.drive, Robot.arm, Robot.shooter, Robot.belt, resetPose, stopAtEnd);
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
   * @param resetPose Whether or not to reset the pose before starting the path.
   * @param stopAtEnd Whether or not to stop the motors at the end of the path.
   */
  public PathweaverIntake(String trajectory, Drivetrain drive, Arm arm, Shooter shooter, Belt belt, boolean resetPose, boolean stopAtEnd) {
    addRequirements(drive, arm, shooter, belt);
    addCommands(
      parallel(
        new PathweaverCommand(trajectory, resetPose, stopAtEnd),
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
