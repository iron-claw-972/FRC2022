package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class AutoConstants {
  public final double kMaxSpeedMetersPerSecond = Units.feetToMeters(1.28); // Max velocity
  public final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(1.28); // Max acceleration

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  // DO NOT MODIFY unless you know what you are doing
  public final double kRamseteB = 2;
  public final double kRamseteZeta = 0.7;

  // 0.2032
  public final double kIntakeDriveDistance = 0.381;
  public final double kDriveSpeed = 0.25;

  // Trajectories should be placed in src/main/deploy/paths
  // name without stuff after . ex AutoPath
  public final String kTrajectoryDirectory = "paths/output/";
  public final String kTrajectoryName = "HangarTest";
  public final String k3BallAuto = "T23Ball";
  public final String[] kAutoPaths = {"HangarTest", "SquareTest", "T13Ball", "T23Ball", "TopAuto", "TwoBallsToHangarTest", "CurveTest"};
}
