package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Rotator;
import frc.robot.util.ClimbUtil;

public class Rotate extends SequentialCommandGroup {
  public Rotate(double angleL, double angleR) {
    this(angleL, angleR, Robot.rotatorL, Robot.rotatorR);
  }

  /**
   * 
   * Sets the setpoint of the climber arms to a specified angle. Do not exceed 125 degrees or go below 90 degrees.
   * 
   * @param angle the angle to rotate to in degrees
   */
  public Rotate(double angleL, double angleR, Rotator rotatorL, Rotator rotatorR) {
    addRequirements(rotatorL, rotatorR);
    addCommands(
      new SequentialCommandGroup(
        // enable the rotator
        new InstantCommand(() -> ClimbUtil.enableRotator()),
        // angle the rotator
        new InstantCommand(() -> ClimbUtil.setAngle(angleL, angleR)),
        // wait until rotator reaches its setpoint
        new WaitUntilCommand(() -> ClimbUtil.isRotatorAtSetpoint())
    ));
    }
}
