package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.commands.cargoCommands.GetDistance;
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
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft");
    System.out.println("Limelight Distance: " + Units.metersToFeet(GetDistance.limelightDistance) + " ft");
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
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft");
    System.out.println("Limelight Distance: " + Units.metersToFeet(GetDistance.limelightDistance) + " ft");

    assertTrue(GetDistance.isFinished);
    assertEquals(GetDistance.optimalVelocity, 24.902942434373344, 0.01);
    assertEquals(GetDistance.optimalStipeAngle, 107.5477713279413, 0.01);
    assertEquals(GetDistance.loggedOptimalShootingAngle, 68.5477713279413, 0.01);
    assertEquals(GetDistance.loggedTargetHeightOffset, 5.939272295035024, 0.01);
    assertEquals(Units.metersToFeet(GetDistance.pivotDistance), 8.307067860841022, 0.01);
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
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft");
    System.out.println("Limelight Distance: " + Units.metersToFeet(GetDistance.limelightDistance) + " ft");

    assertTrue(GetDistance.isFinished);
    assertEquals(GetDistance.optimalVelocity, 25.620514441673492, 0.01);
    assertEquals(GetDistance.optimalStipeAngle, 152.71140547791944, 0.01);
    assertEquals(GetDistance.loggedOptimalShootingAngle, 66.28859452208056, 0.01);
    assertEquals(GetDistance.loggedTargetHeightOffset, 5.855838655049209, 0.01);
    assertEquals(Units.metersToFeet(GetDistance.pivotDistance), 9.751124034891342, 0.01);
  }
  

  @Test
  public void printBackShooting2Results() {
    when(cargoRotator.currentAngle()).thenReturn(140.0);
    doAnswer(stipeAngle -> {
      return Units.feetToMeters(7);
    }).when(limelight).getHubDistance(anyDouble());

    scheduler.schedule(backGetDistance);
    scheduler.run();

    System.out.println("Back shooting 2");
    System.out.println("Is Finished: " + GetDistance.isFinished);
    System.out.println("Optimal Velocity: " + GetDistance.optimalVelocity + " ft/s");
    System.out.println("Optimal Stipe Angle: " + GetDistance.optimalStipeAngle + " deg");
    System.out.println("Acute Optimal Shooting Angle: " + GetDistance.loggedOptimalShootingAngle + " deg");
    System.out.println("Target height offset: " + GetDistance.loggedTargetHeightOffset + " ft");
    System.out.println("Pivot Distance: " + Units.metersToFeet(GetDistance.pivotDistance) + " ft");
    System.out.println("Limelight Distance: " + Units.metersToFeet(GetDistance.limelightDistance) + " ft");
  }
}
