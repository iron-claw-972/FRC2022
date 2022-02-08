package frc.robot.robotConstants.shooterBelt;

import edu.wpi.first.math.util.Units;

public class TraversoBeltConstants {
  public final int kShooterBeltMotorPort = 11;

  public final double kOuttakeSpeed = 2.0;
  public final double kIntakeSpeed = 2.0;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kGearRatio = 1.0;
  public final double kEncoderMetersPerPulse = kWheelDiameterMeters * Math.PI / (double) kEncoderResolution / kGearRatio;

  // PID Stuff
  public final double kShooterBeltP = 0.1;
  public final double kShooterBeltI = 0;
  public final double kShooterBeltD = 0;
  public final double kShooterBeltVelocityPIDTolerance = 0;
}
