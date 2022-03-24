package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.autoCommands.DriveDistance;
import frc.robot.commands.autoCommands.ShootAuto;
import frc.robot.commands.cargoCommands.PositionArm;
import frc.robot.robotConstants.cargoRotator.MarinusCargoRotatorConstants;
import frc.robot.robotConstants.shooterBelt.MarinusBeltConstants;
import frc.robot.robotConstants.shooterWheel.MarinusCargoShooterConstants;

public class Front1BallAuto extends SequentialCommandGroup {
  //THIS AUTO IS NOT TESTED AND MAY BE INACCURATE

  public static MarinusCargoRotatorConstants cargoConstants = new MarinusCargoRotatorConstants();
  public static MarinusBeltConstants beltConstants = new MarinusBeltConstants();
  public static MarinusCargoShooterConstants wheelConstants = new MarinusCargoShooterConstants();

  public Front1BallAuto() {
    addRequirements(RobotContainer.m_drive, RobotContainer.m_cargoBelt, RobotContainer.m_cargoRotator, RobotContainer.m_cargoShooter);
    addCommands(
        new ShootAuto(false, true, 1, () -> true, 108, 22.4719101),
        new DriveDistance(-1.0),
        new PositionArm(154)
    );
  }
}
