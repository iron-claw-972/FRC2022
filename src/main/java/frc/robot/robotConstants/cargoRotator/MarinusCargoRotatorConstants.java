package frc.robot.robotConstants.cargoRotator;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.util.Units;

public class MarinusCargoRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmEncoder = 4;

    // motor clamping
    public final double kMotorClamp = 12;

    // the motor ports of the arm
    public final int kArmMotor = 5;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0.0;

    // the distance allowed from the setpoint (IN DEGREES)
    // Was 3 before
    public final double kArmTolerance = 3;

    // arm characteristics
    public final double kOffset = 0.17114160427854;
    public final double kFeedForward = 0.58;
    public final double kFeedForwardOffsetAngle = 30.0;
    public final double kFeedForwardHardstopTolerance = 3.0;

    public final double kSAngle = -50;

    public final double kFrontLimelightScanPos = 80;
    public final double kBackLimelightScanPos = 146;
    
    // locations
    public final double kIntakePos = 2; // 0 degrees (when on hardstops)
    public final double kBackOuttakeNearPos = 160;
    public final double kBackOuttakeFarPos = 168;
    public final double kFrontOuttakeNearPos = 110; // TODO: Update this value
    public final double kFrontOuttakeFarPos = 85;
    public final double kFrontOuttakeHighPos = 108;
    public final double kBackOuttakeHighPos = 160;
    public final double kBackOuttakeLimelightPos = 168;
    public final double kFrontOuttakeAutoPos = 108;
    public final double kStowPos = 172;
    public final double kAutoBackOuttakeFarPos = 168;
    public final double kUprightPos = 123;

    public final double kPivotToShootingExitPointLength = Units.inchesToMeters(19.313208);
    public final double kStipeToShootingTrajectoryAngularOffset = -39;
    public final double kStipeToPhysicalShooterAngularOffset = -44;
    
    // pid constants
    // public final double kP = 0.1;
    // public final double kI = 0.08;
    // public final double kD = 0.00;
    public final double kP = 0.12;
    public final double kI = 0.08;
    public final double kD = 0.00;

    // motor constants
    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final NeutralMode kNeutral = NeutralMode.Brake;
}
