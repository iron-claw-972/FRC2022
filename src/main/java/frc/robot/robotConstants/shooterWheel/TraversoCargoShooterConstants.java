package frc.robot.robotConstants.shooterWheel;

import edu.wpi.first.math.util.Units;

public class TraversoCargoShooterConstants {
  public final int kCargoShooterMotorPort = 19;

  public final int kFrontOuttakeSpeed = 1;
  public final int kBackOuttakeSpeed = 1;
  public final int kIntakeSpeed = 1;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kGearRatio = 1.0;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;

  // PID Stuff
  public final double kP = 0.004;
  public final double kI = 0.0016;
  public final double kD = 0.0004;
  public final double kForward = 0.0013;
  public final double kVelocityPIDTolerance = 0;
  // Feedforward
  public final double kS = 0;
  public final double kV = 0;
  public final double kA = 0;

  public final double kMotorClamp = 1;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final boolean kCoast = true;
}
