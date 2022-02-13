package frc.robot.robotConstants.cargoArm;

public class TraversoCargoArmConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmEncoder = 1;

    // motor clamping
    public final double kMotorClamp = 0.3;

    // the motor ports of the arm
    public final int kArmMotor = 24;

    public final double kArmMaxDegree = 30;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public final double kArmTolerance = 3;

    //whether the gearbox is flipped
    public final int kFlipped = 1;

    public final double kArmMaxDegreeTicks = kArmMaxDegree / kArmDegreeMultiple;
    
}
