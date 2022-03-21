package frc.robot.robotConstants.climbRotator;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class MarinusClimbRotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmLeftEncoder = 6;
    public final int kArmRightEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.1;

    // the motor ports of the arm
    public final int kArmRightMotor = 9;
    public final int kArmLeftMotor = 8;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 1.5;

    // locations
    public final double kNinetyDeg = 96;
    public final double kMaxBackward = 123;
    public final double kMaxForward = kNinetyDeg;
    public final double kToBar = 115;
    public final double kHookStatic = 110;

    // PID constants
    public final double kP = 0.05;
    public final double kI = 0.00;
    public final double kD = 0.00;

    // encoder offset
    public final double kArmLeftEncoderOffset = 178.39190538479764;
    public final double kArmRightEncoderOffset = 69.99340665758516;
    
    // upper limit switches
    public final int kLeftLimitSwitchUpper = 0;
    public final int kRightLimitSwitchUpper = 0;
    
    // lower limit switches
    public final int kLeftLimitSwitchLower = 7;
    public final int kRightLimitSwitchLower = 2;

    public final double kLimitSwitchDebouncer = 0.01;

    public final double kSupplyCurrentLimit = 30;
    public final double kSupplyTriggerThreshold = 30;
    public final double kSupplyTriggerDuration = 0;
    public final NeutralMode kNeutral = NeutralMode.Brake;

}