package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ClimberMethods;

/**
 * 
 * Extends the climber to a distance using a PID. This enables the extender, sets the extension, and waits until the extension is reached.
 * 
 * @param extension The extension desired for the extenders.
 */
public class ClimbExtenderMove extends SequentialCommandGroup {
  public ClimbExtenderMove(double extension) {
    addRequirements(RobotContainer.m_extenderL, RobotContainer.m_extenderR);
    addCommands(
      new SequentialCommandGroup(
        // enable the extender
        new InstantCommand(() -> ClimberMethods.enableExtender()),
  
        // set the angle of the rotator, arm, and extension of the extender
        new InstantCommand(() -> ClimberMethods.setExtension(extension)),
  
        // wait until they all reach their setpoints
        new WaitUntilCommand(() -> ClimberMethods.isExtenderAtSetpoint())
    ));
    }
}
