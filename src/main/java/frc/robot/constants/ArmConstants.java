package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
    //none of this shit means anything ??????
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
    // public final double kOffset = 0.17114160427854;
    public final double kOffset = -0.159;
    public final double kFeedForward = 0.58;
    public final double kFeedForwardOffsetAngle = 30.0;
    public final double kFeedForwardHardstopTolerance = 3.0;

    public final double kSAngle = -50;

    public final double kFrontLimelightScanPos = 80;
    public final double kBackLimelightScanPos = 172;

    // These should be stipe angles
    public final double kFrontMaxShootingAngle = 134;
    public final double kBackMaxShootingAngle = 157;

    // locations
    public final double kIntakePos = 2; // 0 degrees (when on hardstops)
    public final double kBackOuttakeNearPos = 160;
    public final double kBackOuttakeFarPos = 168;
    public final double kFrontOuttakeNearPos = 110; // TODO: Update this value
    public final double kFrontOuttakeFarPos = 85;

    public final double kFrontOuttakeHighPos = 108;
    public final double kBackOuttakeHighPos = 157;

    public final double kBackOuttakeLimelightPos = 168;
    public final double kFrontOuttakeAutoPos = 108;
    public final double kStowPos = 172;
    public final double kAutoBackOuttakeFarPos = 168;
    public final double kUprightPos = 123;

    public final double kOptimalBackShootingPos = 150;

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

    //sim constants
    //TODO: needs configuration! Got values from cad, likely inaccurate
    public final DCMotor kGearbox = DCMotor.getFalcon500(1); //The type of and number of motors in the arm gearbox.
    public final double kGearRatio = 128.0 / 9.0; // The gearing of the arm (numbers greater than 1 represent reductions).
    public final double kMomentOfInertia = 0.566;// The moment of inertia of the arm, calculated from CAD software. (jKgMetersSquared)
    public final double kLength = 0.710;// The length of the arm. (meters)
    public final double kMinAngleRads = Units.degreesToRadians(2); // The minimum angle that the arm is capable of. (radians)
    public final double kMaxAngleRads = Units.degreesToRadians(172);    // The maximum angle that the arm is capable of. (radians)
    public final double kMassKg = 5.456;    // The mass of the arm. (kilograms)

}
