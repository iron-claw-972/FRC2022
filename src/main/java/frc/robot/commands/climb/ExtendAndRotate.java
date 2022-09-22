package frc.robot.commands.climb;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Extender;

public class ExtendAndRotate extends SequentialCommandGroup {
  /**
   * 
   * Moves both the rotator and extender at the same time to their specified angles and extensions
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ExtendAndRotate(double extensionL, double extensionR, double angleL, double angleR) {
    this(extensionL, extensionR, angleL, angleR, Robot.extenderL, Robot.extenderR);
  }

  /**
   * 
   * Moves both the rotator and extender at the same time to their specified angles and extensions
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ExtendAndRotate(double extensionL, double extensionR, double angleL, double angleR, Extender extenderL, Extender extenderR) {
    addRequirements(extenderL, extenderR);
    addCommands(
      // moves the rotator
      parallel(
        new Rotate(angleL, angleR),
        new Extend(extensionL, extensionR)
      )
    );
  }
}
