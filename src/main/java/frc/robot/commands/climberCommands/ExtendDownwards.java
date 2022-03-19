package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;
import frc.robot.util.ClimberMethods;

public class ExtendDownwards extends SequentialCommandGroup {

  TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();

  /**
   * 
   * In parallel powers each extender downwards until it reaches the bottom limit switch. 
   * Should be used exclusively when you want to go fully down. Has an option to zero when
   * it reaches the bottom as the bottom is exactly where it hits the limit switch, 0.
   * 
   * @param zero whether or not it should zero the extender when it reaches the bottom
   */
  public ExtendDownwards(boolean zero){
      addRequirements(RobotContainer.m_extenderR, RobotContainer.m_extenderL);
      addCommands(
          new InstantCommand(() -> ClimberMethods.disableExtender()), //disable just to make sure PID doesn't run
          parallel(
            //in parallel moves each extender down and then waits until it is compressed
            sequence(
              new InstantCommand(() -> RobotContainer.m_extenderL.setOutput(extend.kDownPower)),
              new WaitUntilCommand(() -> RobotContainer.m_extenderL.compressionLimitSwitch()),
              new InstantCommand(() -> RobotContainer.m_extenderL.disable()),
              (zero ? (new InstantCommand(() -> RobotContainer.m_extenderL.zero())) : new DoNothing())
            ),
            sequence(
              new InstantCommand(() -> RobotContainer.m_extenderR.setOutput(extend.kDownPower)),
              new WaitUntilCommand(() -> RobotContainer.m_extenderR.compressionLimitSwitch()),
              new InstantCommand(() -> RobotContainer.m_extenderR.disable()),
              (zero ? (new InstantCommand(() -> RobotContainer.m_extenderR.zero())) : new DoNothing())
            )
          )
      );
  }
}
