package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public final static int kArmRightEncoder = 0;
    public final static int kArmLeftEncoder = 0;

    // motor clamping
    public final static double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final static int kArmRightMotor = 24;
    public final static int kArmLeftMotor = 24;

    public final static double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final static double kArmDegreeMultiple = 360.0;
    public final static double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final static double kArmTolerance = 0;

    public final static double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;

    public final static double kArmLeftEncoderOffset = 0;
    public final static double kArmRightEncoderOffset = 0;
    
    // locations
    public final static double kNinetyDeg = 90;
    public final static double kMaxBackward = 125;
    public final static double kMaxForward = 80;
}
