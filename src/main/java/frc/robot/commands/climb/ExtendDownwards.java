package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.ClimberMethods;

public class ExtendDownwards extends SequentialCommandGroup {
  /**
   * 
   * In parallel powers each extender downwards until it reaches the bottom limit switch. 
   * Should be used exclusively when you want to go fully down. Has an option to zero when
   * it reaches the bottom as the bottom is exactly where it hits the limit switch, 0.
   * 
   * @param zero whether or not it should zero the extender when it reaches the bottom
   */
  public ExtendDownwards(boolean zero) {
      addRequirements(Robot.m_extenderR, Robot.m_extenderL);
      addCommands(
          new InstantCommand(() -> ClimberMethods.disableExtender()), //disable just to make sure PID doesn't run
          parallel(
            //in parallel moves each extender down and then waits until it is compressed
            compressed(true) ? new DoNothing() : sequence(
              new InstantCommand(() -> Robot.m_extenderL.setOutput(zero ? Constants.extender.kDownPowerCalibration : Constants.extender.kDownPowerNoCalibration)),
              new WaitUntilCommand(() -> compressed(true)),
              new InstantCommand(() -> Robot.m_extenderL.disable()),
              (zero ? (new InstantCommand(() -> Robot.m_extenderL.zero())) : new DoNothing())
            ),
            compressed(false) ? new DoNothing() : sequence(
              new InstantCommand(() -> Robot.m_extenderR.setOutput(zero ? Constants.extender.kDownPowerCalibration : Constants.extender.kDownPowerNoCalibration)),
              new WaitUntilCommand(() -> compressed(false)),
              new InstantCommand(() -> Robot.m_extenderR.disable()),
              (zero ? (new InstantCommand(() -> Robot.m_extenderR.zero())) : new DoNothing())
            )
          )
      );
  }

  public boolean compressed(boolean left) {
    if (left) {
      return Robot.m_extenderL.compressionLimitSwitch();// || Math.abs(Robot.mExtenderL.getVelocity()) < 0.05;
    } else {
      return Robot.m_extenderR.compressionLimitSwitch();// || Math.abs(Robot.mExtenderR.getVelocity()) < 0.05;
    }
  }
}
