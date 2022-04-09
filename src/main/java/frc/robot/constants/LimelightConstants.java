package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class LimelightConstants {
  public final double kAlignPIDTolerance = 1.5;
  public final double kAlignP = 0.2;
  public final double kAlignI = 0.00;
  public final double kAlignD = 0.02;
  // public final double kAlignPIDTolerance = 1.5;
  // public final double kAlignP = 0.2;
  // public final double kAlignI = 0.00;
  // public final double kAlignD = 0.001;

  // public final double kAlignPIDTolerance = 1;
  // public final double kAlignP = 0.037;
  // public final double kAlignI = 0.05;
  // public final double kAlignD = 0.004;
  public final double kAutoThrottlePow = 0.3;

  public final double kTurnPIDTolerance = 1;
  public final double kTurnP = 0.01; // Originally 0.025
  public final double kTurnI = 0;
  public final double kTurnD = 0;
  public final double kMaxTurnPower = 0.6;

  public final double kThrottlePIDTolerance = 0.5;
  public final double kThrottleP = 0.12;
  public final double kThrottleI = 0.08;
  public final double kThrottleD = 0;

  // public final double kAngularFactor = 1.32;
  public final double kAngularFactor = 1;

  public final double kStipeToLimelightFaceAngularOffset = -33;
  public final double kStipeToLimelightPosAngularOffset = 0;
  // public final double kHubHeight = Units.inchesToMeters(102.625); // Distance from ground to center of vision tape
  public final double kHubHeight = Units.inchesToMeters(103); // Distance from ground to center of vision tape
  public final double kHubDiameter = Units.inchesToMeters(60.125);
  public final double kBallTargetHeight = Units.inchesToMeters(9.5/2);
  public final boolean kIsMountedHorizontally = true;

  // public final double kFrontLimelightDistanceFactor = 1.41;
  // public final double kBackLimelightDistanceFactor = 1;
  public final double kFrontLimelightDistanceFactor = 1.259;
  public final double kBackLimelightDistanceFactor = 0.902;

  // Pipeline numbers used by Limelight NT
  public final int kRedCargoPipeline = 0;
  public final int kBlueCargoPipeline = 1;
  public final int kUpperHubPipeline = 2;
  // public final int kUpperHubPipeline = 4;
  public final int kDriverPipeline = 3;

  public final double kPivotToLimelightLength = Units.inchesToMeters(21.07); // Distance from pivot to limelight lens
  public final double kPivotHeight = Units.inchesToMeters(14.0625); // Distance from pivot to ground
}
