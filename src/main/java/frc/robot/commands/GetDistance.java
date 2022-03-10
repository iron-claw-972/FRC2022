package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoRotator;
import frc.robot.subsystems.Limelight;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;
  private final CargoRotator m_cargoRotator;

  public GetDistance(Limelight limelight, CargoRotator cargoRotator) {
    m_limelight = limelight;
    m_cargoRotator = cargoRotator;
    addRequirements(limelight, cargoRotator);
  }

  @Override
  public void execute() {
    // double angle = Units.radiansToDegrees(Math.asin(8.5/22));
    double angle = m_cargoRotator.currentAngle();
    double distance = Units.metersToInches(m_limelight.getHubDistance(angle));
    // SmartDashboard.putNumber("Distance", distance);
    // System.out.println(distance);
  }
}
  