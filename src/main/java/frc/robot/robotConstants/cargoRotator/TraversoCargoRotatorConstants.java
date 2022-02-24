package frc.robot.robotConstants.cargoRotator;

public class TraversoCargoRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmEncoder = 4;

    // motor clamping
    public final double kMotorClamp = 6;

    // the motor ports of the arm
    public final int kArmMotor = 5;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0.0;

    // the distance allowed from the setpoint (IN DECIMAL DEGREES (1 = 360, .5 = 180, .25 = 90))
    public final double kArmTolerance = 3;

    public final double kOffset = 0.17114160427854;
    public final double kFeedForward = 0.58;
    
    // locations
    public final double kIntakePos = 0; // 0 degrees (when on hardstops)
    public final double kBackOutakeNearPos = -1;
    public final double kBackOutakeFarPos = -1;
    public final double kFrontOutakeNearPos = 90; // or -90? or 270?
    public final double kFrontOutakeFarPos = -1;
    public final double kStowPos = -1;
    
    public final double kP = 0.1;
    public final double kI = 0.08;
    public final double kD = 0.00;

    // feed forward constants
    public final double kS = 0;
    public final double kG = 0.15;
    public final double kV = 2.91;
    public final double kA = 0.01;

    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final boolean kCoast = false;
}
