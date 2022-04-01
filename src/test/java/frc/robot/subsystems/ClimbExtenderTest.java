package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.ExtenderConstants;
import frc.robot.util.LimitSwitch;

import org.junit.*;


public class ClimbExtenderTest {
  private LimitSwitch limitSwitch = mock(LimitSwitch.class);
  private WPI_TalonFX motor = mock(WPI_TalonFX.class);
  private ExtenderConstants extend = mock(ExtenderConstants.class);

  private Extender extender = new Extender(true, motor, limitSwitch);

  @Test
  public void ExtenderInherentlyOff() {
    assertFalse(extender.isEnabled()); // is the extender disabled on init?
  }

  @Test
  public void ExtenderIsRight() {
    extender.setSide(false);
    assertEquals("Right", extender.getSide()); // is the extender the right side?
  }

  @Test
  public void ExtenderSetpointSet() {
    extender.enable();
    extender.setGoal(extend.kLeftMaxUpwards);
    extender.periodic();
    assertEquals(extend.kLeftMaxUpwards, extender.getGoal(), 0); // does the extender have its setpoint set?
  }

  @Test
  public void ExtenderZero() {
    extender.zero();
    assertEquals(0, extender.currentExtensionRaw(), 0); // is the extender zeroed?
  }
}