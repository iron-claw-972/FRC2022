package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;
import frc.robot.util.ShooterMethods;

public class FlexibleAuto extends SequentialCommandGroup {
  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();
  
  public FlexibleAuto(boolean isFar, double distance, boolean intakeSecond) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_limelight, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(

        new WaitCommand(2),

        (isFar ? 
            // far shooting
            new Shoot(cargoConstants.kFrontOuttakeAutoPos, beltConstants.kIntakeSpeed, 
                wheelConstants.kFrontOuttakeAutoSpeed, beltConstants.kOuttakeSpeed, true):
            // fender shooting
            new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, 
                -2100, beltConstants.kOuttakeSpeed, false)).withTimeout(8),
        new DriveDistance(distance).withTimeout(1.5),
        new InstantCommand(() -> RobotContainer.m_drive.setCoastMode()),
        (intakeSecond ? 
            new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kBackOuttakeFarPos, true, Constants.kIsRedAlliance).withTimeout(4):
            new DoNothing()),
        (intakeSecond ? 
            new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true).withTimeout(4):
            new DoNothing())
    );
  }
}
