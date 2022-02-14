package frc.robot.robotConstants.climbRotator;

public class TraversoClimbRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmRightEncoder = 1;
    public final int kArmLeftEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = -1;
    public final int kArmLeftMotor = 24;

    public final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 3;

    // whether the gearbox is flipped
    public final int kFlipped = 1;

    // maximum degrees the arm can go in ticks
    public final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;
    
    // locations
    public final double kNinetyDeg = 90;
    public final double kMaxBackward = 135;
    public final double kMaxForward = 80;
    public final double kToBar = 110;
}
