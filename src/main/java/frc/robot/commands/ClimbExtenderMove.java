package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;
import frc.robot.util.ClimberMethods;

/**
 * 
 * Sets the setpoint for the extenders (in ticks). Do not set the setpoint to be a negative number.
 * 
 * @param extension The extension desired for the extenders.
 */
public class ClimbExtenderMove extends SequentialCommandGroup {

  TraversoClimbExtenderConstants extenderConstants = new TraversoClimbExtenderConstants();

  public ClimbExtenderMove(double extensionR, double extensionL) {
    addRequirements(RobotContainer.m_extenderL, RobotContainer.m_extenderR);
    addCommands(
      // if the extenders' setpoints are zero, treat it as a zeroing if it's permitted by kAlwaysZero
      (extensionL == 0 && extensionR == 0 ? new ExtendDownwards(extenderConstants.kAlwaysZero) :
      // if it's not supposed to zero or is not having a setpoint of zero,
      new SequentialCommandGroup(
        // enable the extender
        new InstantCommand(() -> ClimberMethods.enableExtender()),
  
        // set the setpoints of the extenders
        // please note: these extensions are different to account for the left not reaching as high as it should
        new InstantCommand(() -> RobotContainer.m_extenderL.set(extensionL)),
        new InstantCommand(() -> RobotContainer.m_extenderR.set(extensionR)),
  
        // wait until both extenders reach their setpoints
        new WaitUntilCommand(() -> ClimberMethods.isExtenderAtSetpoint())
    )));
    }
}
