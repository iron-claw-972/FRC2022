package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public final int[] rightMotorPorts = {3, 4};
  public final int[] leftMotorPorts = {1, 2};

  //auto might be off because they were calibrated for classbot3

  public final double kTrackWidth = Units.inchesToMeters(31); // Distance between center of left wheel and center of right wheel in meters
  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameter = Units.inchesToMeters(6); // In meters
  public final double kGearRatio = 11.25;
  public final double kWheelCircumference = kWheelDiameter * Math.PI;
  public final double kDistancePerPulse = kWheelCircumference / kGearRatio / (double) kEncoderResolution;

  // Use the SysId program in WPILib Tools to estimate values
  // Drivetrain
  public final double KsLinear = 0.61763; // Ks
  public final double KvLinear = 2.4394; // Kv
  public final double KaLinear = 0.19755; // Ka

  // Drivetrain (Angular)
  public final double KsAngular = 0.68983; // Ks
  public final double KvAngular = 1.5; // Kv
  public final double KaAngular = 0.3; // Ka

  // Position PID gain values
  public final double KpPosition = 82.761; // Kp for Ramsete PID
  public final double KiPosition = 0; // Kp for Ramsete PID
  public final double KdPosition = 5.1379; // Kp for Ramsete PID

  // Velocity PID gain values
  public final double KpVelocity = 2.8662; // Proportional
  public final double KiVelocity = 0; // Integral
  public final double KdVelocity = 0; // Derivative

  public final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

  public final double kSlowSpeed = 0.5;
  public final double kSlewRate = 5;

  // Teleop max speeds
  public final double kMaxSpeedMetersPerSecond = Units.feetToMeters(12); // Max velocity
  public final double kMaxAngularSpeedRadiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(60); // Max angular velocity

  public final LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(
      KvLinear,
      KaLinear,
      KvAngular,
      KaAngular
    );

  public final boolean kRightEncoderReversed = false;
  public final boolean kLeftEncoderReversed = true;


  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 50;
  public final double kSupplyTriggerDuration = 0.3;
  public final NeutralMode kNeutralMode = NeutralMode.Brake;
  public final NeutralMode kMainNeutralMode = NeutralMode.Brake;
}
