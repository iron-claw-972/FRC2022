package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.ClimberMethods;

public class ClimbRotatorMove extends SequentialCommandGroup {

  /**
   * 
   * Sets the setpoint of the climber arms to a specified angle. Do not exceed 125 degrees or go below 90 degrees.
   * 
   * @param angle the angle to rotate to in degrees
   */
  public ClimbRotatorMove(double angle) {
    addRequirements(Robot.m_rotatorL, Robot.m_rotatorR);
    addCommands(
      new SequentialCommandGroup(
        // enable the rotator
        new InstantCommand(() -> ClimberMethods.enableRotator()),
        new PrintCommand("passed enabled"),

        // angle the rotator
        new InstantCommand(() -> ClimberMethods.setAngle(angle)),
  
        new PrintCommand("passed set angle"),

        // wait until rotator reaches its setpoint
        new WaitUntilCommand(() -> ClimberMethods.isRotatorAtSetpoint()),
        new PrintCommand("passed setpoint")
    ));
    }
}