package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.climbExtender.TraversoClimbExtenderConstants;
import frc.robot.subsystems.ClimbExtender;
import frc.robot.util.ClimberMethods;

public class ExtendDownwards extends SequentialCommandGroup {

  TraversoClimbExtenderConstants extend = new TraversoClimbExtenderConstants();

  public ExtendDownwards(){
      addRequirements(RobotContainer.m_extenderR, RobotContainer.m_extenderL);
      addCommands(
          new InstantCommand(() -> ClimberMethods.disableExtender()), //disable just to make sure PID doesn't run
          parallel(
            sequence(
              new InstantCommand(() -> RobotContainer.m_extenderL.setOutput(extend.kDownPower)),
              new WaitUntilCommand(() -> compressed(RobotContainer.m_extenderL)),
              new InstantCommand(() -> RobotContainer.m_extenderL.disable())
            ),
            sequence(
              new InstantCommand(() -> RobotContainer.m_extenderR.setOutput(extend.kDownPower)),
              new WaitUntilCommand(() -> compressed(RobotContainer.m_extenderR)),
              new InstantCommand(() -> RobotContainer.m_extenderR.disable())
            )
          )
      );
  }

  private boolean compressed(ClimbExtender extender) {
    return extender.compressionLimitSwitch();
}


}
