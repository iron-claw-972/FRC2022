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

  public GetDistance frontGetDistance;
  public GetDistance backGetDistance;

  @Before
  public void setup() {
    scheduler = CommandScheduler.getInstance();

    limelight = mock(Limelight.class);
    cargoRotator = mock(CargoRotator.class);

    frontGetDistance = new GetDistance(limelight, cargoRotator);
    backGetDistance = new GetDistance(limelight, cargoRotator);
  }

  @After
  public void cleanup() {
    scheduler.cancelAll();
  }

  @Test
  public void printShootingResults() {
    when(cargoRotator.currentAngle()).thenReturn(64.0);
    doAnswer(stipeAngle -> {
      return Units.feetToMeters(32);
    }).when(limelight).getHubDistance(anyDouble());

    scheduler.schedule(frontGetDistance);
    scheduler.run();

    System.out.println("Front shooting 1");
    System.out.println("Is Finished: " + GetDistance.isFinished);
    System.out.println("Optimal Velocity: " + GetDistance.optimalVelocity + " ft/s");
    System.out.println("Optimal Stipe Angle: " + GetDistance.optimalStipeAngle + " deg");
    System.out.println("Acute Optimal Shooting Angle: " + GetDistance.loggedOptimalShootingAngle + " deg");
    System.out.println("Target height offset: " + GetDistance.loggedTargetHeightOffset + " ft");
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft/s");
  }

  @Test
  public void printFrontShootingResults() {
    when(cargoRotator.currentAngle()).thenReturn(80.0);
    doAnswer(stipeAngle -> {
      return Units.feetToMeters(8);
    }).when(limelight).getHubDistance(anyDouble());

    scheduler.schedule(frontGetDistance);
    scheduler.run();

    System.out.println("Front shooting 2");
    System.out.println("Is Finished: " + GetDistance.isFinished);
    System.out.println("Optimal Velocity: " + GetDistance.optimalVelocity + " ft/s");
    System.out.println("Optimal Stipe Angle: " + GetDistance.optimalStipeAngle + " deg");
    System.out.println("Acute Optimal Shooting Angle: " + GetDistance.loggedOptimalShootingAngle + " deg");
    System.out.println("Target height offset: " + GetDistance.loggedTargetHeightOffset + " ft");
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft/s");

    assertTrue(GetDistance.isFinished);
    assertEquals(GetDistance.optimalVelocity, 25.831072275994146, 0.01);
    assertEquals(GetDistance.optimalStipeAngle, 111.5361014686653, 0.01);
    assertEquals(GetDistance.loggedOptimalShootingAngle, 72.5361014686653, 0.01);
    assertEquals(GetDistance.loggedTargetHeightOffset, 5.892897422527696, 0.01);
    assertEquals(Units.metersToFeet(GetDistance.pivotDistance), 7.692932139158977, 0.01);
  }

  @Test
  public void printBackShootingResults() {
    when(cargoRotator.currentAngle()).thenReturn(172.0);
    doAnswer(stipeAngle -> {
      return Units.feetToMeters(8);
    }).when(limelight).getHubDistance(anyDouble());

    scheduler.schedule(backGetDistance);
    scheduler.run();

    System.out.println("Back shooting");
    System.out.println("Is Finished: " + GetDistance.isFinished);
    System.out.println("Optimal Velocity: " + GetDistance.optimalVelocity + " ft/s");
    System.out.println("Optimal Stipe Angle: " + GetDistance.optimalStipeAngle + " deg");
    System.out.println("Acute Optimal Shooting Angle: " + GetDistance.loggedOptimalShootingAngle + " deg");
    System.out.println("Target height offset: " + GetDistance.loggedTargetHeightOffset + " ft");
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft/s");

    assertTrue(GetDistance.isFinished);
    assertEquals(GetDistance.optimalVelocity, 24.848370528593488, 0.01);
    assertEquals(GetDistance.optimalStipeAngle, 145.8264388645579, 0.01);
    assertEquals(GetDistance.loggedOptimalShootingAngle, 73.1735611354421, 0.01);
    assertEquals(GetDistance.loggedTargetHeightOffset, 5.804937916286363, 0.01);
    assertEquals(Units.metersToFeet(GetDistance.pivotDistance), 6.248875965108656, 0.01);
  }
}
