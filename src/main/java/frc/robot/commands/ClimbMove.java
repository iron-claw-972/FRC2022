package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.ClimberMethods;

public class ClimbMove extends SequentialCommandGroup {
  /**
   * 
   * This enables the rotators and extenders for the climb and waits until they reach their setpoints using a PID.
   * 
   * @param extension the extension of the climber.
   * @param angle the angle of the climb rotator, in degrees.
   */
  public ClimbMove(double extension, double angle) {
    addRequirements(RobotContainer.m_climbRotatorL, RobotContainer.m_climbRotatorR, RobotContainer.m_extenderL, RobotContainer.m_extenderR);
    addCommands(
      new SequentialCommandGroup(
        // enable the rotator, arm, and extender
        new InstantCommand(() -> ClimberMethods.enableRotator()),
        new InstantCommand(() -> ClimberMethods.enableExtender()),
  
        // set the angle of the rotator, arm, and extension of the extender
        new InstantCommand(() -> ClimberMethods.setAngle(angle)),
        new InstantCommand(() -> ClimberMethods.setExtension(extension)),
  
        // wait until they all reach their setpoints
        new WaitUntilCommand(() -> ClimberMethods.isRotatorAtSetpoint()),
        new WaitUntilCommand(() -> ClimberMethods.isExtenderAtSetpoint())
        
    ));
    }
}
