package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public final int[] rightMotorPorts = {13, 0};
  public final int[] leftMotorPorts = {12, 0};

  //auto might be off because they were calibrated for classbot3

  public final double kTrackWidth = Units.inchesToMeters(31); // Distance between center of left wheel and center of right wheel in meters
  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameter = Units.inchesToMeters(6); // In meters
  public final double kGearRatio = 11.25;
  public final double kWheelCircumference = kWheelDiameter * Math.PI;
  public final double kDistancePerPulse = kWheelCircumference / kGearRatio / (double) kEncoderResolution;

  public final double kRobotLength = Units.inchesToMeters(36.5);
  public final double kRobotWidth = Units.inchesToMeters(33.5);

  // Use the SysId program in WPILib Tools to estimate values
  // Drivetrain
  public final double KsLinear = 0.63571; // Ks
  public final double KvLinear = 2.5123; // Kv
  public final double KaLinear = 0.3665; // Ka

  // Drivetrain (Angular)
  public final double KsAngular = 0.68983; // Ks
  public final double KvAngular = 1.5; // Kv
  public final double KaAngular = 0.3; // Ka

  // Position PID gain values
  public final double KpPosition = 92.094; // Kp for Ramsete PID
  public final double KiPosition = 0; // Kp for Ramsete PID
  public final double KdPosition = 7.4545; // Kp for Ramsete PID

  // Velocity PID gain values
  public final double KpVelocity = 3.455; // Proportional
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
