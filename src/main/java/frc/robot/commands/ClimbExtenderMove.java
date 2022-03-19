package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;
import frc.robot.robotConstants.climbRotator.TraversoClimbRotatorConstants;
import frc.robot.util.ClimberMethods;

/**
 * 
 * Extends the climber to a distance using a PID. This enables the extender, sets the extension, and waits until the extension is reached.
 * 
 * @param extension The extension desired for the extenders.
 */
public class ClimbExtenderMove extends SequentialCommandGroup {

  TraversoClimbExtenderConstants extenderConstants = new TraversoClimbExtenderConstants();

  public ClimbExtenderMove(double extensionR, double extensionL) {
    addRequirements(RobotContainer.m_extenderL, RobotContainer.m_extenderR);
    addCommands(
      //if the extension is zero it is better to use this command that uses limit switches
      (extensionL == 0 && extensionR == 0 ? new ExtendDownwards(extenderConstants.kAlwaysZero) :
      new SequentialCommandGroup(
        // enable the extender
        new InstantCommand(() -> ClimberMethods.enableExtender()),
  
        // set the angle of the rotator, arm, and extension of the extender
        new InstantCommand(() -> RobotContainer.m_extenderL.set(extensionL)),
        new InstantCommand(() -> RobotContainer.m_extenderR.set(extensionR)),
  
        // wait until they all reach their setpoints
        new WaitUntilCommand(() -> ClimberMethods.isExtenderAtSetpoint())
    )));
    }
}
