package frc.robot.robotConstants.drivetrain;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

public class TraversoDriveConstants {

  public final int[] rightMotorPorts = {3, 4};
  public final int[] leftMotorPorts = {1, 2};

  //auto might be off because they were calibrated for classbot3

  public final double kTrackWidthMeters = Units.inchesToMeters(23); // Distance between center of left wheel and center of right wheel in meters
  public  final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(6);
  public final double kGearRatio = 11.25;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;
  public final double kEncoderMetersPerSecond = kWheelDiameterMeters * Math.PI / kGearRatio * 10.0;

  // Use the SysId program in WPILib Tools to estimate values
  public final double ksVolts = 0.59765; // Ks
  public final double kvVoltSecondsPerMeter = 2.6544; // Kv
  public final double kaVoltSecondsSquaredPerMeter = 0.15897; // Ka
  public final double kRamseteP = 2.7489; // Kp for Ramsete PID
  public final double kvVoltSecondsPerRadian = 0.05;
  public final double kaVoltSecondsSquaredPerRadian = 0.005;

  public final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
    kvVoltSecondsPerMeter,
    kaVoltSecondsSquaredPerMeter,
    kvVoltSecondsPerRadian,
    kaVoltSecondsSquaredPerRadian
  );


  // Velocity PID gain values
  public final double kVelocityP = 1; // Proportional
  public final double kVelocityI = 0; // Integral
  public final double kVelocityD = 0; // Derivative

  public final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);

  public final boolean kRightEncoderReversed = false;
  public final boolean kLeftEncoderReversed = true;
}
