package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public final static int kArmRightEncoder = 1;
    public final static int kArmLeftEncoder = 0;

    // motor clamping
    public final static double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final static int kArmRightMotor = -1;
    public final static int kArmLeftMotor = 24;

    public final static double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final static double kArmDegreeMultiple = 360.0;
    public final static double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public final static double kArmTolerance = 3;

    // whether the gearbox is flipped
    public final static int kFlipped = 1;

    public final static double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;
    
    // locations
    public final static double kNinetyDeg = 90;
    public final static double kMaxBackward = 135;
    public final static double kMaxForward = 80;
}
