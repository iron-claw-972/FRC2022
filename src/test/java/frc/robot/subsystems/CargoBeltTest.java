package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import org.junit.*;


public class CargoBeltTest {
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public Belt belt = new Belt(motor);

  @Test
  public void BeltInherentlyOff() {
    assertFalse(belt.isEnabled()); // is the belt inherently disabled?

    belt.enable();
    assertTrue(belt.isEnabled()); // is the belt enabled?
  }

  @Test
  public void BeltPowerSet() {
    assertEquals(0.0, belt.getMotorPower(), 0); // is the belt inherently not powered?

    belt.setPower(.5);
    belt.periodic();
    assertEquals(.5, belt.getMotorPower(), 0); // is the belt's power correct?
  }
}