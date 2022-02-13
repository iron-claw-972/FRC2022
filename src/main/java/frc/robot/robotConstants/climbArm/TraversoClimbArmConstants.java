package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public static final int kArmRightEncoder = 1;
    public static final int kArmLeftEncoder = 0;

    // motor clamping
    public static final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public static final int kArmRightMotor = -1;
    public static final int kArmLeftMotor = 24;

    public static final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public static final double kArmDegreeMultiple = 360.0;
    public static final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public static final double kArmTolerance = 3;

    // whether the gearbox is flipped
    public static final int kFlipped = 1;

    public static final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;
    
    // locations
    public static final double kNinetyDeg = 90;
    public static final double kMaxBackward = 135;
    public static final double kMaxForward = 80;
    public static final double kToBar = 110;
}
