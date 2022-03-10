package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.cargoRotator.DonkCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.DonkBeltConstants;
import frc.robot.robotConstants.shooterWheel.DonkCargoShooterConstants;
import frc.robot.util.ShooterMethods;

public class FlexibleAuto extends SequentialCommandGroup {
  public static DonkCargoRotatorConstants cargoConstants = new DonkCargoRotatorConstants();
  public static DonkBeltConstants beltConstants = new DonkBeltConstants();
  public static DonkCargoShooterConstants wheelConstants = new DonkCargoShooterConstants();

  public FlexibleAuto(boolean isFar, double distance, boolean intakeSecond) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_limelight, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        new WaitCommand(2),
        (isFar ? 
          // far shooting
          new Shoot(cargoConstants.kFrontOuttakeAutoPos, beltConstants.kIntakeSpeed, 
              wheelConstants.kFrontOuttakeAutoSpeed, beltConstants.kOuttakeSpeed, true, 0):
          // fender shooting
          new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, 
              -2100, beltConstants.kOuttakeSpeed, false, 0)),
        new DriveDistance(distance).withTimeout(1.5),
        new InstantCommand(() -> RobotContainer.m_drive.setCoastMode()),
        (intakeSecond ? 
            new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kBackOuttakeFarPos, true, Constants.kIsRedAlliance).withTimeout(4):
            new DoNothing()),
        (intakeSecond ? 
            new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true, 0).withTimeout(4):
            new DoNothing())
    );
  }
}
