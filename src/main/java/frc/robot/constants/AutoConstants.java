package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class AutoConstants {
  public final double kMaxSpeedMetersPerSecond = Units.feetToMeters(6.56168); // Max velocity
  public final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(6.56168); // Max acceleration

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  // DO NOT MODIFY unless you know what you are doing
  public final double kRamseteB = 2;
  public final double kRamseteZeta = 0.7;

  // 0.2032
  public final double kIntakeDriveDistance = 0.381;
  public final double kDriveSpeed = 0.5;

  // Pathweaver output folder should be src/main/deploy/paths
  // name without stuff after . ex AutoPath.wpilib.json -> AutoPath
  public final String kTrajectoryDirectory = "paths/output/";
}
