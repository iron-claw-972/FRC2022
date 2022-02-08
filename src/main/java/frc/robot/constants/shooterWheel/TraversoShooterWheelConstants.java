package frc.robot.constants.shooterWheel;

import edu.wpi.first.math.util.Units;

public class TraversoShooterWheelConstants {
  public final int kShooterWheelMotorPort = 11;

  public final int kFrontOuttakeSpeed = 1;
  public final int kBackOuttakeSpeed = 1;
  public final int kIntakeSpeed = 1;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kGearRatio = 1.0;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;

  // PID Stuff
  public final double kShooterWheelP = 0.1;
  public final double kShooterWheelI = 0;
  public final double kShooterWheelD = 0;
  public final double kShooterWheelVelocityPIDTolerance = 0;
}
