package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ClimberMethods;

public class ClimbMove extends SequentialCommandGroup {
  /**
   * 
   * This enables the rotators and extenders for the climb and waits until they reach their setpoints using a PID.
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ClimbMove(double extension, double angle) {
    addCommands(
      parallel(new ClimbRotatorMove(angle), new ClimbExtenderMove(extension))
    );
  }
}
