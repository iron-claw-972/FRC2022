package frc.robot.constants.drivetrain;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class ClassBot3Constants {
    
  public final int[] rightMotorPorts = {10};
  public final int[] leftMotorPorts = {12};

    
  public final double kTrackWidthMeters = Units.inchesToMeters(20); // Distance between center of left wheel and center of right wheel in meters

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kGearRatio = 62.0 / 8.0;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;
  public final double kEncoderMetersPerSecond = kWheelDiameterMeters * Math.PI / kGearRatio * 10.0;

  // Use the SysId program in WPILib Tools to estimate values
  public final double ksVolts = 0.52734; // Ks
  public final double kvVoltSecondsPerMeter = 2.6398; // Kv
  public final double kaVoltSecondsSquaredPerMeter = 0.21913; // Ka
  public final double kRamseteP = 3.1014; // Kp for Ramsete PID
  public final double kvVoltSecondsPerRadian = 0.05;
  public final double kaVoltSecondsSquaredPerRadian = 0.005;

  public final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
    kvVoltSecondsPerMeter,
    kaVoltSecondsSquaredPerMeter,
    kvVoltSecondsPerRadian,
    kaVoltSecondsSquaredPerRadian);


  // Velocity PID gain values
  public final double kVelocityP = 1; // Proportional
  public final double kVelocityI = 0; // Integral
  public final double kVelocityD = 0; // Derivative

  public final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

  public final boolean kRightEncoderReversed = false;
  public final boolean kLeftEncoderReversed = true;
}
