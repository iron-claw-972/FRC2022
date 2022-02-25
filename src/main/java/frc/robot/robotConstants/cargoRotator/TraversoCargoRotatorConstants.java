package frc.robot.robotConstants.cargoRotator;

public class TraversoCargoRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmEncoder = 4;

    // motor clamping
    public final double kMotorClamp = 10;

    // the motor ports of the arm
    public final int kArmMotor = 5;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0.0;

    // the distance allowed from the setpoint (IN DEGREES)
    public final double kArmTolerance = 3;

    // arm characteristics
    public final double kOffset = 0.17114160427854;
    public final double kFeedForward = 0.58;
    public final double kFeedForwardOffsetAngle = 30.0;
    public final double kFeedForwardHardstopTolerance = 3.0;
    
    // locations
    public final double kIntakePos = 2; // 0 degrees (when on hardstops)
    public final double kBackOuttakeNearPos = 132;
    public final double kBackOuttakeFarPos = 147;
    public final double kFrontOuttakeNearPos = 100; // TODO: Update this value
    public final double kFrontOuttakeFarPos = 90;
    public final double kStowPos = 170;
    
    // pid constants
    public final double kP = 0.1;
    public final double kI = 0.08;
    public final double kD = 0.00;

    // motor constants
    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final boolean kCoast = false;
}
