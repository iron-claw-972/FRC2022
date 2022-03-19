package frc.robot.robotConstants.climbRotator;

public class TraversoClimbRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmLeftEncoder = 6;
    public final int kArmRightEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.1;

    // the motor ports of the arm
    public final int kArmRightMotor = 9;
    public final int kArmLeftMotor = 8;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 2;

    // locations
    public final double kNinetyDeg = 95;
    public final double kMaxBackward = 123;
    public final double kMaxForward = kNinetyDeg;
    public final double kToBar = 119;
    public final double kHookStatic = 110;

    // PID constants
    public final double kP = 0.05;
    public final double kI = 0.001;
    public final double kD = 0.00;

    // encoder offset
    public final double kArmLeftEncoderOffset = 475.5449638886241;
    public final double kArmRightEncoderOffset = -113.71123791778095;
    
    // upper limit switches
    public final int kLeftLimitSwitchUpper = 0;
    public final int kRightLimitSwitchUpper = 0;
    
    // lower limit switches
    public final int kLeftLimitSwitchLower = 7;
    public final int kRightLimitSwitchLower = 2;

    public final double kLimitSwitchDebouncer = 0.01;

    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final boolean kCoast = false;

}