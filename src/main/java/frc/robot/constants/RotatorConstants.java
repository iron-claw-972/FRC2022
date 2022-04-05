package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class RotatorConstants {
    // the duty cycle encoder ports of the arm
    public final int kArmLeftEncoder = 6;
    public final int kArmRightEncoder = 0;

    // motor clamping
    public final double kMotorClamp = 0.35;

    // the motor ports of the arm
    public final int kArmRightMotor = 9;
    public final int kArmLeftMotor = 8;

    // Current Tick Value * Degree Multiple = Current Angle
    public final double kArmDegreeMultiple = 360.0;
    public final double kArmZeroEncoderDegrees = 0;

    // the distance allowed from the setpoint (in degrees)
    public final double kArmTolerance = 0.3;

    // locations
    public final double kMiddle = 116;

    public final double kMaxBackwardL = 123.5;
    public final double kMaxForwardL = 96.1;
    public final double kToBarL = 115;

    public final double kMaxBackwardR = 124.5;
    public final double kMaxForwardR = 96;
    public final double kToBarR = 115;

    // PID constants
    public final double kP = 0.01;
    public final double kI = 0.00;
    public final double kD = 0.00;
    public final double kF = 0.06;

    // encoder offset
    public final double kArmLeftEncoderOffset = 177.92214137305353;
    public final double kArmRightEncoderOffset = 2.518796987969921;
    
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