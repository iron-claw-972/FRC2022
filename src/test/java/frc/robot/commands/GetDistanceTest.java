package frc.robot.commands;

import frc.robot.util.ShooterMethods;
import org.junit.*;
import static org.junit.Assert.*;

public class GetDistanceTest {

  @Before
  public void setup() {
  }

  @Test
  public void testCalculateOptimalAngle() {
    assertEquals(ShooterMethods.getOptimalShootingAngle(-69, 4, 2.64), 75.7067529341, 0.01);
  }

  @Test
  public void testCalculateOptimalSpeed() {
    assertEquals(ShooterMethods.getOptimalShooterSpeed(75.7067529341, 2.64, 4), 9.92398647342, 0.01);
  }

}
