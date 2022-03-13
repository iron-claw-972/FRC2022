package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;

public class FlexibleAuto extends SequentialCommandGroup {
  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  public FlexibleAuto(double distance, boolean intakeSecond, boolean shootSecond , boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_limelight, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        parallel(
          new DriveDistance(distance),
          new Shoot(true, false, false, 1, () -> DriveDistance.isFinished)
        ),
        new ConditionalCommand(
          new Intake(cargoConstants.kBackOuttakeFarPos, true, Constants.kIsRedAlliance).withTimeout(1), 
          new DoNothing(),
          () -> intakeSecond
        ),
        new ConditionalCommand(
          parallel(
            new AlignToUpperHub(RobotContainer.m_limelight,RobotContainer.m_drive).withTimeout(1),
            new Shoot(true, false, false, 0, () -> AlignToUpperHub.isFinished)
          ),
          new DoNothing(), 
          () -> intakeSecond && shootSecond
        ),
        new PositionArm(154)
    );
  }
}
