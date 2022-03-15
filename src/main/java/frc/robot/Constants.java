/*
Copyright (c) 2018-2019 FIRST. All Rights Reserved.
Open Source Software - may be modified and shared by FRC teams. The code
must be accompanied by the FIRST BSD license file in the root directory of
the project.
*/

package frc.robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/*
The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
constants.  This class should not be used for any other purpose.  All constants should be
declared globally (i.e. public static).  Do not put anything functional in this class.

<p>It is advised to statically import this class (or one of its inner classes) wherever the
constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kMaxVoltage = 12.0;
  public static final boolean kIsRedAlliance = DriverStation.getAlliance() == Alliance.Red;

  public static final class JoyConstants {
    public static final int kDriverJoy = 0;
    public static final int kOperatorJoy = 1;
    public static final int kOperatorClimbJoy = 2;
    public static final double kDeadband = 0.05;
  }

  public static final class DriveConstants {
    public static final double kSlowSpeed = 0.5;
    public static final double kSlewRate = 3;

    // Teleop max speeds
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(12); // Max velocity
    public static final double kMaxAngularSpeedRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(60); // Max angular velocity
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(3.28); // Max velocity
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(3.28); // Max acceleration

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    // DO NOT MODIFY unless you know what you are doing
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // 0.2032
    public static final double kAutoIntakeDriveDistance = 0.381;

    // Trajectories should be placed in src/main/deploy/paths
    // name without stuff after . ex AutoPath
    public static final String kTrajectoryName = "TopAuto";
  }
}
