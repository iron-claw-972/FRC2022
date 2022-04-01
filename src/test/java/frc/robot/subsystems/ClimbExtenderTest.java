package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.LimitSwitch;

import org.junit.*;


public class ClimbExtenderTest {
  private LimitSwitch limitSwitch = mock(LimitSwitch.class);
  private WPI_TalonFX motor = mock(WPI_TalonFX.class);

  private Extender extender = new Extender(true, motor, limitSwitch);

  @Test
  public void ExtenderInherentlyOff() {
    assertFalse(extender.isEnabled()); // is the extender disabled on init?

    extender.enable();
    assertTrue(extender.isEnabled()); // is the extender enabled?
  }

  @Test
  public void ExtenderIsRight() {
    extender.setSide(false);
    assertEquals("Right", extender.getSide()); // is the extender the right side?
  }

  @Test
  public void ExtenderSetpointSet() {
    extender.setGoal(2000);
    extender.periodic();
    assertEquals(2000, extender.getGoal(), 0); // is the extender's setpoint correct?
  }

  @Test
  public void ExtenderZero() {
    extender.zero();
    assertEquals(0, extender.currentExtensionRaw(), 0); // is the extender zeroed?
  }
}