package frc.robot.robotConstants.cargoArm;

public class TraversoCargoArmConstants {
    // the duty cycle encoder ports of the arm
    public final static int kArmEncoder = 1;

    // motor clamping
    public final static double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final static int kArmMotor = 24;

    public final static double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final static double kArmDegreeMultiple = 360.0;
    public final static double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public final static double kArmTolerance = 3;

    public final static double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;

    public final static double kArmEncoderOffset = 0;
    
    // locations
    public final static double kNinetyDeg = 90;
    public final static double kMaxBackward = 135;
    public final static double kMaxForward = 80;
    
}
