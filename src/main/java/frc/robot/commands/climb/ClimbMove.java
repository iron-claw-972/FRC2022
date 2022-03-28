package frc.robot.commands.climb;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Extender;

public class ClimbMove extends SequentialCommandGroup {
  public ClimbMove(double extensionL, double extensionR, double angle) {
    this(extensionL, extensionR, angle, Robot.extenderL, Robot.extenderR);
  }

  /**
   * 
   * Moves both the rotator and extender at the same time to their specified angles and extensions
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ClimbMove(double extensionL, double extensionR, double angle, Extender extenderL, Extender extenderR) {
    addRequirements(extenderL, extenderR);
    addCommands(
      // moves the rotator
      parallel(new ClimbRotatorMove(angle), new ClimbExtenderMove(extensionL, extensionR))
    );
  }
}
