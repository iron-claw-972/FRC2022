package frc.robot.robotConstants.climbArm;

public class TraversoClimbArmConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmRightEncoder = 0;
    public final int kArmLeftEncoder = 1;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = 24;
    public final int kArmLeftMotor = -1;

    public final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 0;

    //whether the gearbox is flipped
    public final int kFlipped = -1;

    public final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;
    
}
