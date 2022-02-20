package frc.robot.robotConstants.cargoRotator;

public class TraversoCargoRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmEncoder = -1;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmMotor = -1;


    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public final double kArmTolerance = 3;


    public final double kArmEncoderOffset = 0;
    
    // locations
    public final double kNinetyDeg = 90;
    public final double kMaxBackward = 135;
    public final double kMaxForward = 80;
    
    public final double kP = 0.03;
    public final double kI = 0.00;
    public final double kD = 0.00;
    
}
