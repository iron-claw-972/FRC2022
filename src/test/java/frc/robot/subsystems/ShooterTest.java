package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.*;

public class ShooterTest {
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public Shooter shooter = new Shooter(motor);

  @Test
  public void shooterInherentlyOff() {
    assertFalse(shooter.isEnabled()); // is the shooter inherently disabled?

    shooter.enable();
    assertTrue(shooter.isEnabled()); // is the shooter enabled?
  }

  @Test
  public void shooterOutputSet() {
    shooter.setSpeed(5);
    assertEquals(5, shooter.m_shooterPID.getSetpoint(), 0); // is the shooter's setpoint correct?
  }
}