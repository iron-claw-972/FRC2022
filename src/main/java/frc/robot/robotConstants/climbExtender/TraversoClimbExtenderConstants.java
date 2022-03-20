package frc.robot.robotConstants.climbExtender;

public class TraversoClimbExtenderConstants {
  // extending motors (for climber)
  public final int kRightExtenderPort = 10;
  public final int kLeftExtenderPort = 11;

  // the arm's length in ticks
  public final double kSoftLimit = 600000;

  // tolerance allowed to the PID 
  public final double kExtenderTolerance = 400;

  // motor clamps
  public final double kMotorClampDown = -0.9; //should be negative!
  public final double kMotorClampUp = .7;     //should be positive!
  
  // locations
  public final double kLeftMaxUpwards = 546433;
  public final double kLeftHalfway = kLeftMaxUpwards / 2;
  public final double kLeftSlightlyUpward = kLeftMaxUpwards / 4;

  public final double kRightMaxUpwards = 558365;
  public final double kRightHalfway = kRightMaxUpwards / 2;
  public final double kRightSlightlyUpward = kRightMaxUpwards / 4;

  public final double kDownPower = -0.5;
  
  // extender limit switches
  public final int kExtLeftLimitSwitch = 8;
  public final int kExtRightLimitSwitch = 1;

  public final double kExtLimitSwitchDebouncer = 0.01;
  
  // off load PID constants
  public final double kP = 0.0001;
  public final double kI = 0.00;
  public final double kD = 0.00;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final boolean kCoast = false;

  public final boolean kAlwaysZero = false;
}