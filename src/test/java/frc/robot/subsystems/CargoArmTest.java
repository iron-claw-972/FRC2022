package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.*;
import org.junit.*;


public class CargoArmTest {
  public DutyCycleEncoder encoder = mock(DutyCycleEncoder.class);
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public Arm arm = new Arm(encoder, motor);

  @Test
  public void testSetpoint() {
    arm.setPosition(145);
    assertEquals(145, arm.getSetpoint(), 0);
  }

  // @Test
  // public void testDisabled() {
  //   assertFalse(arm.isEnabled());
  // }

}