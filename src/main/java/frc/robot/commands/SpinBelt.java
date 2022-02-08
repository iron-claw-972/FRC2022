package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterBelt;

public class SpinBelt extends CommandBase {
  
  private ShooterBelt m_shooterBelt;
  private double motorSpeed;

  public SpinBelt(ShooterBelt shooterBelt, double speed) {
    m_shooterBelt = shooterBelt;
    motorSpeed = speed;
    addRequirements(shooterBelt);
  }
  
  @Override
  public void execute() {
    if (m_shooterBelt.reachedSetpoint(motorSpeed)){
      m_shooterBelt.stop();
    } else {
      m_shooterBelt.updatePID();
    }
  }
}
