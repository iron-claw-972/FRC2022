package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CargoRotator;
import frc.robot.subsystems.Limelight;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import org.junit.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class GetDistanceTest {
  CommandScheduler scheduler = null;

  public Limelight limelight;
  public CargoRotator cargoRotator;

  public GetDistance getDistance;

  @Before
  public void setup() {
    scheduler = CommandScheduler.getInstance();

    limelight = mock(Limelight.class);
    cargoRotator = mock(CargoRotator.class);

    getDistance = new GetDistance(limelight, cargoRotator);

    when(cargoRotator.currentAngle()).thenReturn(80.0);
    doAnswer(stipeAngle -> {
      return Units.feetToMeters(8);
    }).when(limelight).getHubDistance(anyDouble());

    scheduler.schedule(getDistance);
    scheduler.run();
  }

  @After
  public void cleanup() {
    scheduler.cancelAll();
  }

  @Test
  public void printResults() {
    System.out.println("Is Finished: " + GetDistance.isFinished);
    System.out.println("Optimal Velocity: " + GetDistance.optimalVelocity + " ft/s");
    System.out.println("Optimal Stipe Angle: " + GetDistance.optimalStipeAngle + " deg");
    System.out.println("Optimal Shooting Angle: " + GetDistance.loggedOptimalShootingAngle + " deg");
    System.out.println("Target height offset: " + GetDistance.loggedTargetHeightOffset + " ft");
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft/s");
  }

  @Test
  public void testGetDistanceIsFinished() {
    assertTrue(GetDistance.isFinished);
  }
}
