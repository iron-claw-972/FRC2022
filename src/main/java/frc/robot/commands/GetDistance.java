package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;

  public GetDistance(Limelight subsystem) {
    m_limelight = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // double angle = Units.radiansToDegrees(Math.asin(8.5/22));
    double angle = 0;
    double distance = Units.metersToInches(m_limelight.getHubDistance(angle));
    System.out.println("Distance: " + distance);
  }
}
  