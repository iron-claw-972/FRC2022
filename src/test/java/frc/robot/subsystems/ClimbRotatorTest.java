package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import static org.junit.Assert.assertEquals;

import frc.robot.constants.RotatorConstants;

import org.junit.*;

public class ClimbRotatorTest {
  RotatorConstants rotate = mock(RotatorConstants.class);
  
  public Rotator rotator = new Rotator(true);

  // TODO: Figure out why this is throwing an AllocationException error.
  // This theoretically works.

  @Test
  public void RotatorInherentlyOn() {
    assertTrue(rotator.isEnabled()); // is the rotator disabled on init
  }

  @Test
  public void RotatorIsRight() {
    rotator.setSide(false);
    assertEquals("Right", rotator.getSide());
  }
  
  @Test
  public void RotatorSetpointSet() {
    rotator.enable();
    rotator.setGoal(rotate.kMaxBackward);
    rotator.periodic();
    assertEquals(rotate.kMaxBackward, rotator.getGoal(), 0); // does the rotator have its setpoint set?
  }

  @Test
  public void RotatorReachedSetpoint() {
    rotator.periodic();
    assertTrue(rotator.reachedSetpoint()); // does the rotator reach its setpoint?
  }
}
