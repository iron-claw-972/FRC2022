package frc.robot.commands.cargo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Limelight;
import frc.robot.util.ShooterMethods;

public class GetDistance extends CommandBase {
  private final Limelight mLimelight;
  private final CargoArm m_cargoRotator;

  public static boolean isFinished = false;
  public static double optimalVelocity = Double.NaN;
  public static double optimalStipeAngle = Double.NaN;
  public static double loggedOptimalShootingAngle = Double.NaN;
  public static double loggedTargetHeightOffset = Double.NaN;
  public static double pivotDistance = Double.NaN;
  public static double limelightDistance = Double.NaN;

  private boolean isFront = true;

  public GetDistance(Limelight limelight, CargoArm cargoRotator) {
    mLimelight = limelight;
    m_cargoRotator = cargoRotator;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    isFinished = false;
    mLimelight.setUpperHubPipeline();
    optimalVelocity = Double.NaN;
    optimalStipeAngle = Double.NaN;
    loggedOptimalShootingAngle = Double.NaN;
    loggedTargetHeightOffset = Double.NaN;
    pivotDistance = Double.NaN;
    limelightDistance = Double.NaN;
  }

  @Override
  public void execute() {
    // Get distance from limelight
    double currentStipeAngle = m_cargoRotator.currentAngle(); // From 0 to 175 deg
    
    double currentLimelightFaceAngle = currentStipeAngle + Constants.ll.kStipeToLimelightFaceAngularOffset; // Offset is negative
    double currentLimelightPosAngle = currentStipeAngle + Constants.ll.kStipeToLimelightPosAngularOffset; // Offset is negative
    double currentPhysicalShooterAngle = currentStipeAngle + Constants.arm.kStipeToPhysicalShooterAngularOffset; // Offset is negative

    isFront = currentLimelightFaceAngle < 90; // Uses limelight angle because distance calculation is done with the limelight angle

    // Turns all angles into acute angles for easy calculations
    if (!isFront) {
      currentLimelightFaceAngle = 180 - currentLimelightFaceAngle;
      currentPhysicalShooterAngle = 180 - currentPhysicalShooterAngle;
    }
    
    // Get horizontal distance from vision tape to limelight lens
    limelightDistance = mLimelight.getHubDistance(currentStipeAngle);

    // if (isFront) {
    //   limelightDistance += SmartDashboard.getNumber("Front Distance Error", Constants.ll.kFrontLimelightDistanceError);
    // } else {
    //   limelightDistance += SmartDashboard.getNumber("Back Distance Error", Constants.ll.kBackLimelightDistanceError);
    // }

    if (Double.isNaN(limelightDistance) || ((currentLimelightFaceAngle < 90) != (currentPhysicalShooterAngle < 90))) {
      // If distance not found or limelight on opposite side of shooting trajectory, then do not shoot
      optimalVelocity = Double.NaN;
      optimalStipeAngle = Double.NaN;
      loggedOptimalShootingAngle = Double.NaN;
      pivotDistance = Double.NaN;
      limelightDistance = Double.NaN;
      return;
    }

    pivotDistance = ShooterMethods.limelightDistanceToPivotDistance(limelightDistance, currentLimelightPosAngle);

    double currentTargetHeightOffset = ShooterMethods.getTargetHeightOffset(currentPhysicalShooterAngle);
    double currentShootingDistance = ShooterMethods.getShootingDistance(pivotDistance, currentPhysicalShooterAngle);

    // Find optimal shooting angle
    double optimalShootingAngle = ShooterMethods.getOptimalShootingAngle(Constants.arm.kSAngle, currentShootingDistance, currentTargetHeightOffset);

    // Clamp arm angles that will hit the hex shaft attached to the climb triangle
    
    //// Use this if not working

    // if (isFront) {
    //   optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, (0 - Constants.arm.kStipeToPhysicalShooterAngularOffset), 90);
    // } else {
    //   optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, (0 - Constants.arm.kStipeToPhysicalShooterAngularOffset), 180-(162 + Constants.arm.kStipeToPhysicalShooterAngularOffset));
    // }

    if (isFront) {
      optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, (0 + Constants.arm.kStipeToShootingTrajectoryAngularOffset), (Constants.arm.kFrontMaxShootingAngle + Constants.arm.kStipeToShootingTrajectoryAngularOffset));
    } else {
      optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, (0 - Constants.arm.kStipeToShootingTrajectoryAngularOffset), 180 - (Constants.arm.kBackMaxShootingAngle + Constants.arm.kStipeToShootingTrajectoryAngularOffset));
    }

    // optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, (0 - Constants.arm.kStipeToPhysicalShooterAngularOffset), 61.5);
    // optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, 0, (Constants.arm.kBackMaxShootingAngle + Constants.arm.kStipeToShootingTrajectoryAngularOffset));
    // optimalShootingAngle = MathUtil.clamp(optimalShootingAngle, 45, 130);

    loggedOptimalShootingAngle = optimalShootingAngle; // This is for unit tests

    // Actual shooting angle relative to front zero degrees
    double actualOptimalShootingAngle = (isFront ? optimalShootingAngle : 180 - optimalShootingAngle);
    // SmartDashboard.putNumber("Optimal shooting angle", optimalShootingAngle);

    optimalStipeAngle = actualOptimalShootingAngle - Constants.arm.kStipeToShootingTrajectoryAngularOffset;

    double actualOptimalPhysicalShooterAngle = optimalStipeAngle + Constants.arm.kStipeToPhysicalShooterAngularOffset;
    double optimalPhysicalShooterAngle = (isFront ? actualOptimalPhysicalShooterAngle : 180 - actualOptimalPhysicalShooterAngle);

    // double actualOptimalLimelightAngle = optimalStipeAngle + Constants.ll.kStipeToLimelightAngularOffset;
    // double optimalLimelightAngle = (isFront ? actualOptimalLimelightAngle : 180 - actualOptimalLimelightAngle);

    double newTargetHeightOffset = ShooterMethods.getTargetHeightOffset(optimalPhysicalShooterAngle);
    double newShootingDistance = ShooterMethods.getShootingDistance(pivotDistance, optimalPhysicalShooterAngle);

    loggedTargetHeightOffset = Units.metersToFeet(newTargetHeightOffset); // For unit tests

    // Find optimal shooting velocity
    optimalVelocity = Units.metersToFeet(ShooterMethods.getOptimalShooterSpeed(optimalShootingAngle, newTargetHeightOffset, newShootingDistance));

    // Use shot efficiencies to adjust for error in any calculations in velocity to RPM formula
    if (actualOptimalShootingAngle < 90) {
      optimalVelocity *= SmartDashboard.getNumber("Front Shot Efficiency", Constants.shooter.kFrontShotEfficiency);
    } else {
      optimalVelocity *= SmartDashboard.getNumber("Back Shot Efficiency", Constants.shooter.kBackShotEfficiency);
    }

    isFinished = true;
  }

  @Override
  public boolean isFinished() {
    return !(Double.isNaN(optimalStipeAngle) || Double.isNaN(optimalVelocity)) || isFinished;
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
  