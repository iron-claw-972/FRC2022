/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kMaxVoltage = 12.0;

  public static final class JoyConstants {
    public static final int kDriverJoy = 0;
    public static final int kOperatorJoy = 0;
    public static final double kJoystickDeadband = 0.06; // How much of joystick is "dead" zone [0,1]
  }

  public static final class DriveConstants {
    // Drivetrain motor ports, use -1 for unused motors
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 3;

    public static final int kRightMotor1Port = 0;
    public static final int kRightMotor2Port = 2;

    public static final double kSpeedSlewRateLimit = 5;
    public static final double kRotationSlewRateLimit = 5;

    public static final double kTrackWidthMeters = Units.inchesToMeters(20); // Distance between center of left wheel and center of right wheel in meters

    public static final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kGearRatio = 62.0 / 8.0;
    public static final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;
    public static final double kEncoderMetersPerSecond = kWheelDiameterMeters * Math.PI / kGearRatio * 10.0;

    // Use the SysId program in WPILib Tools to estimate values
    public static final double ksVolts = 0.59765; // Ks
    public static final double kvVoltSecondsPerMeter = 2.6544; // Kv
    public static final double kaVoltSecondsSquaredPerMeter = 0.15897; // Ka
    public static final double kRamseteP = 2.7489; // Kp for Ramsete PID
    public static final double kvVoltSecondsPerRadian = 0.05;
    public static final double kaVoltSecondsSquaredPerRadian = 0.005;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);


    // Velocity PID gain values
    public static final double kVelocityP = 1; // Proportional
    public static final double kVelocityI = 0; // Integral
    public static final double kVelocityD = 0; // Derivative

    // Teleop max speeds
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(15); // Max velocity
    public static final double kMaxAngularSpeedRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(90); // Max angular velocity

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

    public static final boolean kRightEncoderReversed = false;
    public static final boolean kLeftEncoderReversed = true;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 23;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPort = 1;
    public static final int kFrontOutakeSpeed = 1;
    public static final int kBackOutakeSpeed = 1;
    public static final double kFarMultiplier = 1.5;
    public static final int kIntakeSpeed = 1;

    public static final int kMinimumBallProximity = 250;

    // PID Stuff
    public static final double kShooterP = 1;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0;
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
}
