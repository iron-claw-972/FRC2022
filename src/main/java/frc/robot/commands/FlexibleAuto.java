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

  public FlexibleAuto(boolean isFar, double distance, boolean intakeSecond, boolean shootSecond , boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_limelight, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        parallel(
          sequence(
            new InstantCommand(() -> RobotContainer.m_drive.setBrakeMode()),
            new DriveDistance(distance)
          )
        //   (isFar ? 
        //     // far shooting
        //     new Shoot(cargoConstants.kFrontOuttakeAutoPos, beltConstants.kIntakeSpeed, 
        //         wheelConstants.kFrontOuttakeAutoSpeed, beltConstants.kOuttakeSpeed, false, 1):
        //     // fender shooting
        //     new Shoot(cargoConstants.kBackOuttakeNearPos, beltConstants.kIntakeSpeed, 
        //         -2100, beltConstants.kOuttakeSpeed, false, 1))
        )
        // (intakeSecond ? 
        //     new Intake(cargoConstants.kIntakePos, beltConstants.kIntakeSpeed, wheelConstants.kIntakeSpeed, cargoConstants.kBackOuttakeFarPos, true, Constants.kIsRedAlliance).withTimeout(4):
        //     new DoNothing()),
        // (intakeSecond ? 
        //     new ChaseBall(RobotContainer.m_limelight, RobotContainer.m_drive, isRedBall).withTimeout(4):
        //     new DoNothing()),
        //far needs to be false for shooting to work or else we need turn around thing
        // (shootSecond ? 
        //     new AlignToUpperHub(RobotContainer.m_limelight,RobotContainer.m_drive).withTimeout(4):
        //     new DoNothing()),
        // (shootSecond ? 
        //     new Shoot(cargoConstants.kBackOuttakeFarPos, beltConstants.kIntakeSpeed, ShooterMethods.getOptimalShooterSpeed(), beltConstants.kOuttakeSpeed, true, 0).withTimeout(4):
        //     new DoNothing()),
        // new PositionArm(154),
        // new InstantCommand(() -> RobotContainer.m_drive.setCoastMode())
    );
  }
}
