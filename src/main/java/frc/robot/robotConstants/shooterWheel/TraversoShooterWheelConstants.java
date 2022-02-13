package frc.robot.robotConstants.shooterWheel;

import edu.wpi.first.math.util.Units;

public class TraversoShooterWheelConstants {
  public final int kShooterWheelMotorPort = 7;

  public final double kFrontOuttakeSpeed = 1;
  public final double kBackOuttakeSpeed = 1;
  public final double kIntakeSpeed = -0.3;

  public final int kEncoderResolution = 1; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kGearRatio = 22/40;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;

  // PID Stuff
  public final double kP = 0.1;
  public final double kI = 0;
  public final double kD = 0;
  public final double kVelocityPIDTolerance = 0;
  // Feedforward
  public final double kS = 0;
  public final double kV = 0;
  public final double kA = 0;
}
