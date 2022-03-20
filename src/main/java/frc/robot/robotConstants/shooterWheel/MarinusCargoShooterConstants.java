package frc.robot.robotConstants.shooterWheel;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class MarinusCargoShooterConstants {

  public final int kCargoShooterMotorPort = 7;

  public final int kFrontOuttakeFarSpeed = -2900;
  public final int kBackOuttakeFarSpeed = -2180;

  public final int kFrontOuttakeHighSpeed = -2900;
  public final int kBackOuttakeHighSpeed = -2900;

  public final int kFrontOuttakeNearSpeed = -1400;
  public final int kBackOuttakeNearSpeed = -1500; //70 degrees

  public final int kFrontOuttakeAutoSpeed = -2900;
  
  public final int kIntakeSpeed = 2000;

  public final int kEncoderResolution = 2048; // 2048 for Falcon500 integrated encoder
  public final double kDistancePerPulse = 100.0 / kEncoderResolution;
  public final double kGearRatio = (double) 18 / 30;

  public final double kShotEfficiency = 1;


  // PID Stuff
  public final double kP = 0.008;
  public final double kI = 0;
  public final double kD = 0;
  public final double kForward = 0.0018;
  public final double kVelocityPIDTolerance = 15;

  // Feedforward
  public final double kS = 1.0734;
  public final double kV = 0.24225;
  public final double kA = 0.02524;

  public final LinearSystem<N1, N1, N1> kFlywheelPlant =
    LinearSystemId.identifyVelocitySystem(
      kV,
      kA
    );

  public final DCMotor kFlywheelGearbox = DCMotor.getFalcon500(1);

  public final double kMotorClamp = 1;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final NeutralMode kNeutral = NeutralMode.Coast;
}