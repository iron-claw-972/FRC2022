package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Extender;
import frc.robot.util.ClimbUtil;

public class Extend extends SequentialCommandGroup {
  /**
   * 
   * Sets the setpoint for the extenders, or if it is zero, uses Retract() command to power downwards until the limit switch.
   * 
   * @param extensionL The extension desired for the left extender (in ticks).
   * @param extensionR The extension desired for the right extender (in ticks).
   */
  public Extend(double extensionL, double extensionR) {
    this(extensionL, extensionR, Robot.extenderL, Robot.extenderR);
  }

  /**
   * 
   * Sets the setpoint for the extenders, or if it is zero, uses Retract() command to power downwards until the limit switch.
   * 
   * @param extensionL The extension desired for the left extender (in ticks).
   * @param extensionR The extension desired for the right extender (in ticks).
   */
  public Extend(double extensionL, double extensionR, Extender extenderL, Extender extenderR) {
    addRequirements(extenderL, extenderR);
    addCommands(
      // if the extenders' setpoints are zero, treat it as a zeroing if it's permitted by kAlwaysZero
      (extensionL <= 0 && extensionR <= 0 ? new Retract(Constants.extender.kAlwaysZero) :
      // if it's not supposed to zero or is not having a setpoint of zero,
      new SequentialCommandGroup(
        // enable the extender
        new InstantCommand(() -> ClimbUtil.enableExtender()),
  
        // set the setpoints of the extenders
        // please note: these extensions are different to account for the left not reaching as high as it should
        new InstantCommand(() -> extenderL.setGoal(extensionL)),
        new InstantCommand(() -> extenderR.setGoal(extensionR)),
  
        // wait until both extenders reach their setpoints
        new WaitUntilCommand(() -> ClimbUtil.isExtenderAtSetpoint())
    )));
    }
}
