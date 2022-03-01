package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
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
        (isFar ? 
            // far shooting
            new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, 
                ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true):
            // fender shooting
            new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, 
                wheelConstants.kBackOuttakeNearSpeed, beltConstants.kOuttakeSpeed, false)),
        new DriveDistance(distance),
        (intakeSecond ? 
            new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kBackOuttakeFarPos, true, Constants.kIsRedAlliance):
            new DoNothing()),
        (intakeSecond ? 
            new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true):
            new DoNothing())
    );
  }
}
