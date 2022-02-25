package frc.robot.robotConstants.limelight;

import edu.wpi.first.math.util.Units;

public class TraversoLimelightConstants {
  public final double kPivotToLimelightAngleDifference = -33;
  public final double kHubTargetHeight = Units.inchesToMeters(104);
  public final double kBallTargetHeight = Units.inchesToMeters(9.5/2);
  public final boolean kIsMountedHorizontally = true;

  // Pipeline numbers used by Limelight NT
  public final int kRedCargoPipeline = 0;
  public final int kBlueCargoPipeline = 1;
  public final int kUpperHubPipeline = 2;

  public final double kPivotToLimelightDistance = Units.inchesToMeters(21.22); // Distance from pivot to limelight lens
  public final double kPivotHeight = Units.inchesToMeters(13); // Distance from pivot to ground
}
