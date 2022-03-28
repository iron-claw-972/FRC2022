package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.*;
import org.junit.*;


public class CargoArmTest {
  public DutyCycleEncoder encoder = mock(DutyCycleEncoder.class);
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public CargoArm cargoArm = new CargoArm(encoder, motor);

  @Test
  public void testSetpoint() {
    cargoArm.setPosition(145);
    assertEquals(145, cargoArm.getSetpoint(), 0);
  }
}