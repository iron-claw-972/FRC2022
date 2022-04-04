package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class ShooterConstants {

  public final int kCargoShooterMotorPort = 7;

  public final int kFrontOuttakeFarSpeed = -2900;
  public final int kBackOuttakeFarSpeed = -3100;

  public final int kFrontOuttakeHighSpeed = -2900;
  public final int kBackOuttakeHighSpeed = -3357; // 24 ft/s

  public final int kFrontOuttakeNearSpeed = -1400;
  public final int kBackOuttakeNearSpeed = -1500; //70 degrees

  public final int kFrontOuttakeAutoSpeed = -2900;
  
  public final int kIntakeSpeed = 2000;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  // public final double kGearRatio = (double) 18 / 30;
  public final double kGearRatio = 1.0;
  // public final double kDistancePerPulse = 2.0 * Math.PI / kEncoderResolution; // This converts from encoder ticks to rotations
  public final double kDistancePerPulse = 100.0 / kEncoderResolution; // This converts from encoder ticks to rotations

  public final double kFrontShotEfficiency = 1;
  public final double kBackShotEfficiency = 1;
  // public final double kBackShotEfficiency = 0.93;


  // PID Stuff
  // public final double kP = 0.008;
  public final double kP = 0.006;
  public final double kI = 0;
  public final double kD = 0.0004;
  // public final double kForward = 0.0018;
  public final double kForward = 1.07;
  public final double kVelocityTolerance = 30.0;

  // Feedforward
  public final double kS = 1.0917;
  public final double kV = 0.024284;
  public final double kA = 0.002929;

  // public final double kS = 1.1199;
  // public final double kV = 0.012827;
  // public final double kA = 0.0018137;

  public final LinearSystem<N1, N1, N1> kFlywheelPlant = LinearSystemId.identifyVelocitySystem(kV, kA);

  public final DCMotor kFlywheelGearbox = DCMotor.getFalcon500(1);

  public final double kMotorClamp = 1;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final NeutralMode kNeutral = NeutralMode.Coast;
}