package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;

  public static TraversoLimelightConstants constants = new TraversoLimelightConstants();

  public static boolean isFinished = false;
  public static double optimalVelocity = Double.NaN;
  public static double optimalStipeAngle = Double.NaN;
  public static double distance = Double.NaN;

  private boolean isFront = true;

  public GetDistance(Limelight limelight) {
    m_limelight = limelight;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    isFinished = false;
    m_limelight.setUpperHubPipeline();
    optimalVelocity = Double.NaN;
    optimalStipeAngle = Double.NaN;
    distance = Double.NaN;
  }

  @Override
  public void execute() {
    // Get distance from limelight
    double stipeAngle = ShooterMethods.getArmAngle(); // From 0 to 175 deg
    double limelightAngle = stipeAngle + constants.kStipeToLimelightAngularOffset; // Offset is negative
    double physicalShooterAngle = stipeAngle + RobotContainer.cargoConstants.kStipeToPhysicalShooterAngularOffset; // Offset is negative

    isFront = limelightAngle < 90; // Uses limelight angle because distance calculation is done with the limelight angle
    if (!isFront) {
      limelightAngle = 180 - limelightAngle;
      physicalShooterAngle = 180 - physicalShooterAngle;
    }
    
    // Get horizontal distance from vision tape to limelight lens
    distance = m_limelight.getHubDistance(stipeAngle);

    if (Double.isNaN(distance) || ((limelightAngle < 90) != (physicalShooterAngle < 90))) {
      // If distance not found or limelight on opposite side of shooting trajectory, then do not shoot
      optimalVelocity = Double.NaN;
      optimalStipeAngle = Double.NaN;
      distance = Double.NaN;
      return;
    }

    // Now we can be sure that we are working with acute angles

    // We want to work in degrees
    double limelightAngleRad = Units.degreesToRadians(limelightAngle);
    double physicalShooterAngleRad = Units.degreesToRadians(physicalShooterAngle);

    // Horizontal distance from shooter exit point to center of hub
    double shootingDistance = distance // Distance from vision tape to limelight lens
                            + (constants.kHubWidth / 2) // Radius of the hub
                            + (constants.kPivotToLimelightLength * Math.cos(limelightAngleRad)) // Horizontal distance from limelight to stipe pivot
                            - (RobotContainer.cargoConstants.kPivotToShootingExitPointLength * Math.cos(physicalShooterAngleRad)); // Subtract horizontal distance from stipe pivot to exit point of shooter (the midpoint between the centers of the two shooter wheels)

    double targetHeightOffset = constants.kHubHeight // Height of hub
                              - constants.kPivotHeight // Height of stipe pivot
                              - (RobotContainer.cargoConstants.kPivotToShootingExitPointLength * Math.sin(physicalShooterAngleRad)); // Height from pivot to shooter exit point

    // Find optimal shooting angle
    double optimalShootingAngle = ShooterMethods.getOptimalShootingAngle(RobotContainer.cargoConstants.kSAngle, shootingDistance, targetHeightOffset);

    // Physical shooting angle relative to front zero degrees
    double actualOptimalShootingAngle = (isFront ? optimalShootingAngle : 180 - optimalShootingAngle);

    optimalStipeAngle = actualOptimalShootingAngle - RobotContainer.cargoConstants.kStipeToShootingTrajectoryAngularOffset;

    // Find optimal shooting velocity
    optimalVelocity = Units.metersToFeet(ShooterMethods.getOptimalShooterSpeed(optimalShootingAngle, targetHeightOffset, distance));
    optimalVelocity *= -1; // Shooter takes negative input to outtake
    optimalVelocity *= RobotContainer.wheelConstants.kShotEfficiency;
  }

  @Override
  public boolean isFinished() {
    return !(Double.isNaN(optimalStipeAngle) || Double.isNaN(optimalVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = true;
  }
}
  