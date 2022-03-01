package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.ClimberMethods;

public class ClimberMove extends SequentialCommandGroup {
  public ClimberMove(double extension, double angle) {
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
