package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ClimberMethods;

public class ClimbRotatorMove extends SequentialCommandGroup {
  public ClimbRotatorMove(double angle) {
    addRequirements(RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR);
    addCommands(
      new SequentialCommandGroup(
        // enable the rotator
        new InstantCommand(() -> ClimberMethods.enableRotator()),
  
        // angle the rotator
        new InstantCommand(() -> ClimberMethods.setAngle(angle)),
  
        // wait until rotator reaches its setpoint
        new WaitUntilCommand(() -> ClimberMethods.isRotatorAtSetpoint())
    ));
    }
}
