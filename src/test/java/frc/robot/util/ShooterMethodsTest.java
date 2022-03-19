package frc.robot.util;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import edu.wpi.first.wpilibj.*;
import frc.robot.RobotContainer;

import org.junit.*;

public class ShooterMethodsTest {
  @Test
  public void testCalculateOptimalSpeed() {
    assertEquals(ShooterMethods.getOptimalShooterSpeed(75.7067529341, 2.64, 4), 9.92398647342, 0.01);
  }

  @Test
  public void testCalculateOptimalAngle() {
    assertEquals(ShooterMethods.getOptimalShootingAngle(-69, 4, 2.64), 75.7067529341, 0.01);
  }

  @Test
  public void testTargetHeightOffset() {
    assertEquals(ShooterMethods.getTargetHeightOffset(80), RobotContainer.limelightConstants.kHubHeight - (0.3571875 + 0.483102844), 0.01);
  }
}
