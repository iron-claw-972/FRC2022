package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;

public class BackFlexibleAuto extends SequentialCommandGroup {
  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();

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
