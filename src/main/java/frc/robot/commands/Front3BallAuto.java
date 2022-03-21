package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.commands.autoCommands.DriveDistance;
import frc.robot.commands.autoCommands.IntakeAuto;
import frc.robot.commands.autoCommands.ShootAuto;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;
import frc.robot.util.ShooterMethods;

public class Front3BallAuto extends SequentialCommandGroup {
  //THIS AUTO IS NOT TESTED AND MAY BE INACCURATE
  
  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();

  public Front3BallAuto(boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 154, 25)
        ),
        new IntakeAuto(cargoConstants.kBackOuttakeFarPos, false, isRedBall, Constants.AutoConstants.kAutoIntakeDriveDistance),
        new ShootAuto(false, false, 0, () -> true, 154, 25),
        new PositionArm(cargoConstants.kIntakePos),
        parallel(
          Pathweaver.pathweaverCommand(Constants.AutoConstants.k3BallAuto),
          new InstantCommand(() -> ShooterMethods.setWheelRPM(wheelConstants.kIntakeSpeed)),
          new InstantCommand(() -> ShooterMethods.setBeltPower(beltConstants.kIntakeSpeed)),
          sequence(
            new WaitUntilCommand(() -> ShooterMethods.isBallContained()),
            new PositionArm(cargoConstants.kFrontOuttakeAutoPos)
          )
        ),
        new ShootAuto(false, true, 0, () -> true, cargoConstants.kFrontOuttakeAutoPos, wheelConstants.kFrontOuttakeAutoSpeed),
        new PositionArm(154)
    );
  }
}
