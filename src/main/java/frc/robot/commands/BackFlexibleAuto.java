package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;

public class BackFlexibleAuto extends SequentialCommandGroup {
  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  public BackFlexibleAuto(double distance, boolean intakeSecond, boolean shootSecond , boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        parallel(
          new DriveDistance(distance),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 154, 25)
        ),
        new ConditionalCommand(
          new IntakeAuto(cargoConstants.kAutoBackOuttakeFarPos, false, Constants.kIsRedAlliance, Constants.AutoConstants.kAutoIntakeDriveDistance), 
          new DoNothing(),
          () -> intakeSecond
        ),
        new ConditionalCommand(
          new ShootAuto(false, false, 0, () -> true, 154, 25),
          new DoNothing(), 
          () -> intakeSecond && shootSecond
        ),
        new PositionArm(154)
    );
  }
}
