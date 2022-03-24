package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.robotConstants.climbExtender.MarinusClimbExtenderConstants;
import frc.robot.util.ClimberMethods;

public class ExtendDownwards extends SequentialCommandGroup {

  MarinusClimbExtenderConstants extend = new MarinusClimbExtenderConstants();

  /**
   * 
   * In parallel powers each extender downwards until it reaches the bottom limit switch. 
   * Should be used exclusively when you want to go fully down. Has an option to zero when
   * it reaches the bottom as the bottom is exactly where it hits the limit switch, 0.
   * 
   * @param zero whether or not it should zero the extender when it reaches the bottom
   */
  public ExtendDownwards(boolean zero) {
      addRequirements(RobotContainer.m_extenderR, RobotContainer.m_extenderL);
      addCommands(
          new InstantCommand(() -> ClimberMethods.disableExtender()), //disable just to make sure PID doesn't run
          parallel(
            //in parallel moves each extender down and then waits until it is compressed
            compressed(true) ? new DoNothing() : sequence(
              new InstantCommand(() -> RobotContainer.m_extenderL.setOutput(zero ? extend.kDownPowerCalibration : extend.kDownPowerNoCalibration)),
              new WaitUntilCommand(() -> compressed(true)),
              new InstantCommand(() -> RobotContainer.m_extenderL.disable()),
              (zero ? (new InstantCommand(() -> RobotContainer.m_extenderL.zero())) : new DoNothing())
            ),
            compressed(false) ? new DoNothing() : sequence(
              new InstantCommand(() -> RobotContainer.m_extenderR.setOutput(zero ? extend.kDownPowerCalibration : extend.kDownPowerNoCalibration)),
              new WaitUntilCommand(() -> compressed(false)),
              new InstantCommand(() -> RobotContainer.m_extenderR.disable()),
              (zero ? (new InstantCommand(() -> RobotContainer.m_extenderR.zero())) : new DoNothing())
            )
          )
      );
  }

  public boolean compressed(boolean left) {
    if (left) {
      return RobotContainer.m_extenderL.compressionLimitSwitch();// || Math.abs(RobotContainer.m_extenderL.getVelocity()) < 0.05;
    } else {
      return RobotContainer.m_extenderR.compressionLimitSwitch();// || Math.abs(RobotContainer.m_extenderR.getVelocity()) < 0.05;
    }
  }
}
