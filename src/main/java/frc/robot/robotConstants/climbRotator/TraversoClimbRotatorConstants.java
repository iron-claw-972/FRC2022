package frc.robot.robotConstants.climbRotator;

public class TraversoClimbRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmLeftEncoder = 6;
    public final int kArmRightEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = 9;
    public final int kArmLeftMotor = 8;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 1;

    // locations
    public final double kNinetyDeg = 90;
    public final double kMaxBackward = 120;
    public final double kMaxForward = 80;
    public final double kToBar = 100;

    // off load PID constants
    public final double kOffLoadP = 0.02;
    public final double kOffLoadI = 0.00;
    public final double kOffLoadD = 0.00;

    // on load PID constants
    public final double kOnLoadP = 0.08;
    public final double kOnLoadI = 0.00;
    public final double kOnLoadD = 0.00;

    // encoder offset
    public final double kArmLeftEncoderOffset = 237.4268519356713;
    public final double kArmRightEncoderOffset = -114.25816485645413;
    
    // upper limit switches
    public final int kLeftLimitSwitchUpper = 8;
    public final int kRightLimitSwitchUpper = 1;
    
    // lower limit switches
    public final int kLeftLimitSwitchLower = 7;
    public final int kRightLimitSwitchLower = 2;

    public final double kLimitSwitchDebouncer = 0.01;

    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final boolean kCoast = false;

}