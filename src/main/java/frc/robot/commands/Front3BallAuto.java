package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autonomous.drivetrain.Pathweaver;
import frc.robot.robotConstants.cargoRotator.TraversoCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import frc.robot.robotConstants.shooterWheel.TraversoCargoShooterConstants;

public class Front3BallAuto extends SequentialCommandGroup {
  //THIS AUTO IS NOT TESTED AND MAY BE INACCURATE
  
  public static TraversoCargoRotatorConstants cargoConstants = new TraversoCargoRotatorConstants();
  public static TraversoBeltConstants beltConstants = new TraversoBeltConstants();
  public static TraversoCargoShooterConstants wheelConstants = new TraversoCargoShooterConstants();

  public Front3BallAuto(boolean isRedBall) {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        parallel(
          new DriveDistance(0.6642),
          new ShootAuto(false, false, 1, () -> DriveDistance.isFinished, 154, 25)
        ),
        new IntakeAuto(cargoConstants.kBackOuttakeFarPos, false, isRedBall, Constants.AutoConstants.kAutoIntakeDriveDistance),
        new ShootAuto(false, false, 0, () -> true, 154, 25),
        new PositionArm(154),
        Pathweaver.pathweaverCommand(Constants.AutoConstants.k3BallAuto)
    );
  }
}
