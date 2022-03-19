package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.robotConstants.limelight.TraversoLimelightConstants;
import frc.robot.subsystems.CargoRotator;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class GetDistance extends CommandBase {
  private final Limelight m_limelight;
  private final CargoRotator m_cargoRotator;

  public static TraversoLimelightConstants constants = new TraversoLimelightConstants();

  public static boolean isFinished = false;
  public static double optimalVelocity = Double.NaN;
  public static double optimalStipeAngle = Double.NaN;
  public static double loggedOptimalShootingAngle = Double.NaN;
  public static double loggedTargetHeightOffset = Double.NaN;
  public static double pivotDistance = Double.NaN;

  private boolean isFront = true;

  public GetDistance(Limelight limelight, CargoRotator cargoRotator) {
    m_limelight = limelight;
    m_cargoRotator = cargoRotator;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    isFinished = false;
    m_limelight.setUpperHubPipeline();
    optimalVelocity = Double.NaN;
    optimalStipeAngle = Double.NaN;
    loggedOptimalShootingAngle = Double.NaN;
    loggedTargetHeightOffset = Double.NaN;
    pivotDistance = Double.NaN;
  }

  @Override
  public void execute() {
    // Get distance from limelight
    double currentStipeAngle = m_cargoRotator.currentAngle(); // From 0 to 175 deg
    
    double currentLimelightFaceAngle = currentStipeAngle + constants.kStipeToLimelightFaceAngularOffset; // Offset is negative
    double currentLimelightPosAngle = currentStipeAngle + constants.kStipeToLimelightPosAngularOffset; // Offset is negative
    double currentPhysicalShooterAngle = currentStipeAngle + RobotContainer.cargoConstants.kStipeToPhysicalShooterAngularOffset; // Offset is negative

    isFront = currentLimelightFaceAngle < 90; // Uses limelight angle because distance calculation is done with the limelight angle

    // Turns all angles into acute angles for easy calculations
    if (!isFront) {
      currentLimelightFaceAngle = 180 - currentLimelightFaceAngle;
      currentPhysicalShooterAngle = 180 - currentPhysicalShooterAngle;
    }
    
    // Get horizontal distance from vision tape to limelight lens
    double limelightDistance = m_limelight.getHubDistance(currentStipeAngle);

    if (Double.isNaN(limelightDistance) || ((currentLimelightFaceAngle < 90) != (currentPhysicalShooterAngle < 90))) {
      // If distance not found or limelight on opposite side of shooting trajectory, then do not shoot
      optimalVelocity = Double.NaN;
      optimalStipeAngle = Double.NaN;
      loggedOptimalShootingAngle = Double.NaN;
      pivotDistance = Double.NaN;
      return;
    }

    pivotDistance = ShooterMethods.limelightDistanceToPivotDistance(limelightDistance, currentLimelightPosAngle);

    double currentTargetHeightOffset = ShooterMethods.getTargetHeightOffset(currentPhysicalShooterAngle);
    double currentShootingDistance = ShooterMethods.getShootingDistance(pivotDistance, currentPhysicalShooterAngle);

    // Find optimal shooting angle
    double optimalShootingAngle = ShooterMethods.getOptimalShootingAngle(RobotContainer.cargoConstants.kSAngle, currentShootingDistance, currentTargetHeightOffset);
    loggedOptimalShootingAngle = optimalShootingAngle; // This is for unit tests

    // Actual shooting angle relative to front zero degrees
    double actualOptimalShootingAngle = (isFront ? optimalShootingAngle : 180 - optimalShootingAngle);

    optimalStipeAngle = actualOptimalShootingAngle - RobotContainer.cargoConstants.kStipeToShootingTrajectoryAngularOffset;

    double actualOptimalPhysicalShooterAngle = optimalStipeAngle + RobotContainer.cargoConstants.kStipeToPhysicalShooterAngularOffset;
    double optimalPhysicalShooterAngle = (isFront ? actualOptimalPhysicalShooterAngle : 180 - actualOptimalPhysicalShooterAngle);

    // double actualOptimalLimelightAngle = optimalStipeAngle + RobotContainer.limelightConstants.kStipeToLimelightAngularOffset;
    // double optimalLimelightAngle = (isFront ? actualOptimalLimelightAngle : 180 - actualOptimalLimelightAngle);

    double newTargetHeightOffset = ShooterMethods.getTargetHeightOffset(optimalPhysicalShooterAngle);
    double newShootingDistance = ShooterMethods.getShootingDistance(pivotDistance, optimalPhysicalShooterAngle);

    loggedTargetHeightOffset = Units.metersToFeet(newTargetHeightOffset); // For unit tests

    // Find optimal shooting velocity
    optimalVelocity = Units.metersToFeet(ShooterMethods.getOptimalShooterSpeed(optimalShootingAngle, newTargetHeightOffset, newShootingDistance));
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

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
  