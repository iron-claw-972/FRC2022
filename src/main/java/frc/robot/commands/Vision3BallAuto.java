
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autoCommands.DriveDistance;
import frc.robot.commands.autoCommands.DriveRotation;
import frc.robot.commands.autoCommands.ShootAuto;
import frc.robot.commands.cargoCommands.ChaseBall;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.commands.cargoCommands.Shoot;
import frc.robot.commands.autoCommands.IntakeAuto;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;
import frc.robot.util.ShooterMethods;

public class Vision3BallAuto extends SequentialCommandGroup {
  //THIS AUTO IS NOT TESTED AND MAY BE INACCURATE

  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();

  public Vision3BallAuto(boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        new InstantCommand(() -> ShooterMethods.setBeltPower(0)),
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 157, 25)
        ),
        new IntakeAuto(cargoConstants.kAutoBackOuttakeFarPos, false, isRedBall, Constants.AutoConstants.kAutoIntakeDriveDistance), 
        new ShootAuto(false, false, 0, () -> true, 157, 25),
        new DriveRotation(0.65),
        new InstantCommand(() -> ShooterMethods.enableAll()),
        parallel(
          new DriveDistance(1.25),
          new InstantCommand(() -> ShooterMethods.setWheelRPM(wheelConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(beltConstants.kIntakeSpeed))
        ),
        new PositionArm(cargoConstants.kIntakePos),
        new ChaseBall(RobotContainer.m_limelight, RobotContainer.m_drive, isRedBall, false).withTimeout(2),
        new PositionArm(cargoConstants.kFrontOuttakeAutoPos),
        new DriveRotation(-0.6),
        //new ShootAuto(false, true, 0, () -> true, cargoConstants.kFrontOuttakeAutoPos, wheelConstants.kFrontOuttakeAutoSpeed),
        new Shoot(false, true, false, 154, -3357),
        new PositionArm(154)
    );
  }
}
