package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterWheel;

public class SpinShooter extends CommandBase {
  
  private ShooterWheel m_shooterWheel;
  private double motorSpeed;

  public SpinShooter(ShooterWheel shooterWheel, double speed) {
    m_shooterWheel = shooterWheel;
    motorSpeed = speed;
    addRequirements(shooterWheel);
  }
  
  @Override
  public void execute() {
    if (m_shooterWheel.reachedSetpoint(motorSpeed)){
      m_shooterWheel.stop();
    } else {
      m_shooterWheel.updatePID();
    }
  }
}
