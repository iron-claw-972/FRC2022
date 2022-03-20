package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ClimberMethods;

public class ClimbRotatorMove extends SequentialCommandGroup {

  /**
   * 
   * Rotates the climber to an angle using a PID. This enables the rotator, sets the angle, and waits until the angle is reached.
   * 
   * @param angle the angle to rotate to in degrees
   */
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
