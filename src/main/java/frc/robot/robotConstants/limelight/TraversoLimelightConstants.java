package frc.robot.robotConstants.limelight;

import edu.wpi.first.math.util.Units;

public class TraversoLimelightConstants {
  public final double kHubTargetHeight = Units.inchesToMeters(45);
  public final double kBallTargetHeight = Units.inchesToMeters(9.5/2);
  public final boolean kIsMountedHorizontally = true;

  public final int kRedCargoPipeline = 0;
  public final int kBlueCargoPipeline = 1;
  public final int kUpperHubPipeline = 2;

  public final double kPivotToLimelightDistance = Units.inchesToMeters(0);
  public final double kPivotHeight = Units.inchesToMeters(4.5);
}
