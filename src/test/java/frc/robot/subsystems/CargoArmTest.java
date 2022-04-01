package frc.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.*;

import org.junit.*;


public class CargoArmTest {
  public DutyCycleEncoder dce = mock(DutyCycleEncoder.class);
  public WPI_TalonFX motor = mock(WPI_TalonFX.class);

  public Arm arm = new Arm(dce, motor);


  @Test
  public void ArmInherentlyOff() {
    assertFalse(arm.isEnabled()); // is the arm inherently off?
  }

  @Test
  public void ArmSetpointSet() {
    arm.enable();
    assertTrue(arm.isEnabled()); // is the arm enabled?

    arm.setPosition(5);
    arm.periodic();
    assertEquals(5, arm.m_armPID.getSetpoint(), 0); // is the arm's setpoint correct?

    arm.setPosition(-100);
    arm.periodic();
    assertEquals(0, arm.m_armPID.getSetpoint(), 0); // does the clamp work?
  }

  @Test
  public void ArmMathCheck() {
    assertEquals(-0.1736481776669303, arm.cosineOfAngle(100), 0); // does this math return correctly?
  }
}