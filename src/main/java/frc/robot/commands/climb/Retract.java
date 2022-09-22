package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Extender;
import frc.robot.util.ClimbUtil;

public class Retract extends SequentialCommandGroup {
  public Extender m_extenderL, m_extenderR;

  /**
   * 
   * In parallel powers each extender downwards until it reaches the bottom limit switch. 
   * Should be used exclusively when you want to go fully down. Has an option to zero when
   * it reaches the bottom as the bottom is exactly where it hits the limit switch, 0.
   * 
   * @param zero whether or not it should zero the extender when it reaches the bottom
   */
  public Retract(boolean zero) {
    this(zero, Robot.extenderL, Robot.extenderR);
  }
  
  /**
   * 
   * In parallel powers each extender downwards until it reaches the bottom limit switch. 
   * Should be used exclusively when you want to go fully down. Has an option to zero when
   * it reaches the bottom as the bottom is exactly where it hits the limit switch, 0.
   * 
   * @param zero whether or not it should zero the extender when it reaches the bottom
   */
  public Retract(boolean zero, Extender extenderL, Extender extenderR) {
      m_extenderL = extenderL;
      m_extenderR = extenderR;
      addRequirements(extenderR, extenderL);

      addCommands(
          new InstantCommand(() -> ClimbUtil.disableExtender()), //disable just to make sure PID doesn't run
          parallel(
            //in parallel moves each extender down and then waits until it is compressed
            new ConditionalCommand(
              new DoNothing(), 
              sequence(
                new InstantCommand(() -> extenderL.setOutput(zero ? Constants.extender.kDownPowerCalibration : Constants.extender.kDownPowerNoCalibration)),
                new WaitUntilCommand(() -> compressed(true)),
                new InstantCommand(() -> extenderL.disable()),
                (zero ? (new InstantCommand(() -> extenderL.zero())) : new DoNothing())
              ), 
              () -> compressed(true)),
            
            new ConditionalCommand(
              new DoNothing(), 
              sequence(
                new InstantCommand(() -> extenderR.setOutput(zero ? Constants.extender.kDownPowerCalibration : Constants.extender.kDownPowerNoCalibration)),
                new WaitUntilCommand(() -> compressed(false)),
                new InstantCommand(() -> extenderR.disable()),
                (zero ? (new InstantCommand(() -> extenderR.zero())) : new DoNothing())
              ), 
              () -> compressed(false))
          )
      );
  }

  public boolean compressed(boolean left) {
    if (left) {
      return m_extenderL.compressionLimitSwitch();// || Math.abs(mExtenderL.getVelocity()) < 0.05;
    } else {
      return m_extenderR.compressionLimitSwitch();// || Math.abs(mExtenderR.getVelocity()) < 0.05;
    }
  }
}
