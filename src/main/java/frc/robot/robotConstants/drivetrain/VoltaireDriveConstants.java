package frc.robot.robotConstants.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class VoltaireDriveConstants {
  public final int[] rightMotorPorts = {14, 15};
  public final int[] leftMotorPorts = {22, 18};

  public final double KsLinear = 0.60877; // Ks
  public final double KvLinear = 0.049031; // Kv
  public final double KaLinear = 0.0058764; // Ka

  // Drivetrain (Angular) (Copied from other bot!)
  public final double KsAngular = 0.68983; // Ks
  public final double KvAngular = 183.65; // Kv
  public final double KaAngular = 20.98; // Ka

  // Position PID gain values
  public final double KpPosition = 2.1573; // Kp for Ramsete PID
  public final double KiPosition = 0; // Kp for Ramsete PID
  public final double KdPosition = 0.1426; // Kp for Ramsete PID

  // Velocity PID gain values
  public final double KpVelocity = 0.065062; // Proportional
  public final double KiVelocity = 0; // Integral
  public final double KdVelocity = 0; // Derivative

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

  public final double kAutoDriveSpeed = 0.4;

  public final double kTrackWidth = Units.inchesToMeters(23.5); // Distance between center of left wheel and center of right wheel in meters
  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameter = Units.inchesToMeters(6.125); // In meters
  public final double kGearRatio = 8.91;
  public final double kWheelCircumference = kWheelDiameter * Math.PI;
  public final double kDistancePerPulse = kWheelCircumference / kGearRatio / (double) kEncoderResolution;

  public final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
}
