package frc.robot.robotConstants.shooterWheel;

public class TraversoCargoShooterConstants {

  public final int kCargoShooterMotorPort = 7;

  public final int kFrontOuttakeFarSpeed = -2050;
  public final int kBackOuttakeFarSpeed = -2150;

  public final int kFrontOuttakeNearSpeed = -1500;
  public final int kBackOuttakeNearSpeed = -1500; //70 degrees
  
  public final int kIntakeSpeed = 1500;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kDistancePerPulse = 100.0 / kEncoderResolution;

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
