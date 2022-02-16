package frc.robot.robotConstants.climbRotator;

public class TraversoClimbRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmRightEncoder = 1;
    public final int kArmLeftEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmRightMotor = 24;
    public final int kArmLeftMotor = -1;


    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 3;


    
    // locations
    public final double kNinetyDeg = 90;
    public final double kMaxBackward = 135;
    public final double kMaxForward = 80;
    public final double kToBar = 110;

    // off load PID constants
    public final double kOffLoadP = 0.02;
    public final double kOffLoadI = 0.00;
    public final double kOffLoadD = 0.00;

    // on load PID constants
    public final double kOnLoadP = 0.02;
    public final double kOnLoadI = 0.00;
    public final double kOnLoadD = 0.00;

    // encoder offset
    public final double kArmLeftEncoderOffset = 0;
    public final double kArmRightEncoderOffset = 0;
}
