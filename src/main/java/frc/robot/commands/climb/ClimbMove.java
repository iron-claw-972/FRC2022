package frc.robot.commands.climb;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbMove extends SequentialCommandGroup {
  /**
   * 
   * Moves both the rotator and extender at the same time to their specified angles and extensions
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ClimbMove(double extensionR, double extensionL, double angle) {
    addCommands(
      // moves the rotator
      parallel(new ClimbRotatorMove(angle), new ClimbExtenderMove(extensionR, extensionL))
    );
  }
}
