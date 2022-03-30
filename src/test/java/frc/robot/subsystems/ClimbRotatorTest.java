package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.junit.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.RotatorConstants;

public class ClimbRotatorTest {
  private DutyCycleEncoder dce = mock(DutyCycleEncoder.class);
  private WPI_TalonFX motor = mock(WPI_TalonFX.class);
  private RotatorConstants rotate = mock(RotatorConstants.class);
  
  private Rotator rotator = new Rotator(true, dce, motor);

  // TODO: Figure out why this is throwing an AllocationException error.
  // This theoretically works.

  @Test
  public void RotatorInherentlyOn() {
    assertTrue(rotator.isEnabled()); // is the rotator enabled on init?
  }

  @Test
  public void RotatorIsRight() {
    rotator.setSide(false);
    assertEquals("Right", rotator.getSide()); // is the rotator the right side?
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
    System.out.println(rotator.reachedSetpoint());
    System.out.println(rotator.currentAngle());
    assertTrue(rotator.reachedSetpoint()); // does the rotator reach its setpoint?
  }
}
