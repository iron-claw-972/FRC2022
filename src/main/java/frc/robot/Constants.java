/*
Copyright (c) 2018-2019 FIRST. All Rights Reserved.
Open Source Software - may be modified and shared by FRC teams. The code
must be accompanied by the FIRST BSD license file in the root directory of
the project.
*/

package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/*
The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
constants.  This class should not be used for any other purpose.  All constants should be
declared globally (i.e. public static).  Do not put anything functional in this class.

<p>It is advised to statically import this class (or one of its inner classes) wherever the
constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kMaxVoltage = 12.0;

  public static final class JoyConstants {
    public static final int kDriverJoy = 0;
    public static final int kOperatorJoy = 1;
    public static final double kDeadband = 0.05;
  }

  public static final class DriveConstants {
    public static final double kSlowSpeed = 0.5;
    public static final double kSlewRate = 3;

    // Teleop max speeds
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(10); // Max velocity
    public static final double kMaxAngularSpeedRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(60); // Max angular velocity
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(3.28); // Max velocity
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(3.28); // Max acceleration

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    // DO NOT MODIFY unless you know what you are doing
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Trajectories should be placed in src/main/deploy/paths
    // name without stuff after . ex AutoPath
    public static final String kTrajectoryName = "HangarTest";
  }

  public static final class ArmConstants {
    
    public static final int kLeftMotorPort = 24;
    public static final int kRightMotorPort = 24;


    public static final double kGearRatio = 1.0;
    public static final int kEncoderResolution = 4096; // 2048 for Falcon500 integrated encoder

    // public static final double kGearRatio = 162.0 / 1.0;
    // public static final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
    

    
    public static final double kEncoderRotationsPerPulse = 1 / (double) kEncoderResolution / kGearRatio;
    public static final double kEncoderRadiansPerPulse = 2 * Math.PI / (double) kEncoderResolution / kGearRatio;
    

    public static final double kRotatorInchesPerRotation = Math.PI * 6;
    public static final double kRotatorGearRatio = 40.1;
    //Dante said two different gear ratios, I think 40.1 is the one being used? Idk leaving this one here just incase.
    //public static final double kRotatorGearRatio = 20.25;
    public static final double kRotatorSetpoint = 0;
    public static final double kRotatorTicksPerRotation = 1/2048;
    public static final double kRotatorTolerance = 5;

    public static final double kRotatorTickMultiple = (kRotatorTicksPerRotation * kRotatorGearRatio * kRotatorInchesPerRotation);
    public static final PIDController rotatorPID = new PIDController(1, 0, 0);
    public static final double kRotatorMaxArmTicks = 171;
  }
}
