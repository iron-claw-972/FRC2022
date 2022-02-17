package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmRightEncoder = 0;
    public final int kArmLeftEncoder = 1;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = 24;
    public final int kArmLeftMotor = 24;

    public final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 0;

    public final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;

    public final double kArmLeftEncoderOffset = 0;
    public final double kArmRightEncoderOffset = 0;
    
    // locations
    public final double kNinetyDeg = 90;
    public final double kMaxBackward = 125;
    public final double kMaxForward = 80;

    // off load PID constants
    public final double kOffLoadP = 0.02;
    public final double kOffLoadI = 0.00;
    public final double kOffLoadD = 0.00;

    // on load PID constants
    public final double kOnLoadP = 0.02;
    public final double kOnLoadI = 0.00;
    public final double kOnLoadD = 0.00;

    // upper limit switches
    public final int kLeftLimitSwitchUpper = 3;
    public final int kRightLimitSwitchUpper = 6;
    
    // lower limit switches
    public final int kLeftLimitSwitchLower = 7;
    public final int kRightLimitSwitchLower = 8;

    public final double kLimitSwitchDebouncer = 0.01;

}
