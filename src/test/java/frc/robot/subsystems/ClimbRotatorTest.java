// package frc.robot.subsystems;

// import static org.junit.Assert.*;
// import static org.junit.Assert.assertEquals;
// import static org.mockito.Mockito.mock;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import org.junit.*;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;

// public class ClimbRotatorTest {
//   private DutyCycleEncoder dce = mock(DutyCycleEncoder.class);
//   private WPI_TalonFX motor = mock(WPI_TalonFX.class);

//   private Rotator rotator = new Rotator(true, dce, motor);

//   // TODO: Figure out why this is throwing an AllocationException error.
//   // This theoretically works.

//   @Test
//   public void rotatorInherentlyOn() {
//     assertTrue(rotator.isEnabled()); // is the rotator enabled on init?
//   }

//   @Test
//   public void rotatorIsRight() {
//     rotator.setSide(false);
//     assertEquals("Right", rotator.getSide()); // is the rotator the right side?
//   }

//   @Test
//   public void rotatorSetpointSet() {
//     rotator.enable();
//     rotator.setGoal(120);
//     rotator.periodic();
//     assertEquals(120, rotator.getGoal(), 0); // is the rotator's setpoint correct?
//   }
// }
