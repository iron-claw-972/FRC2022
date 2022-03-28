package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.*;
import frc.robot.util.LimitSwitch;

import org.junit.*;


public class ExtenderTest {
  public LimitSwitch limitSwitch = mock(LimitSwitch.class);
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public Extender extender = new Extender(true, motor, limitSwitch);

  @Test
  public void testSetpoint() {
    extender.enable();
    extender.set(500000);
    extender.periodic();
    assertEquals(500000, extender.m_extenderPID.getSetpoint(), 0);
  }

}