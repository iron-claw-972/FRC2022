package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmLeftEncoder = 0;
    public final int kArmRightEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = 24;
    public final int kArmLeftMotor = 24;

    public final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    // public double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 0;

    public final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;

    public final double kArmLeftEncoderOffset = 0;
    public final double kArmRightEncoderOffset = 0;
    
}
