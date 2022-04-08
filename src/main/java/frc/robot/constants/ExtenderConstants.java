package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ExtenderConstants {
  // extending motors (for climber)
  public final int kRightExtenderPort = 10;
  public final int kLeftExtenderPort = 11;

  // the arm's length in ticks
  public final double kSoftLimit = 600000;

  // tolerance allowed to the PID 
  public final double kExtenderTolerance = 550;

  // motor clamps
  public final double kMotorClampDown = -0.9; //should be negative! (not used)
  public final double kMotorClampUp = 1.0;     //should be positive!
  
  // locations
  public final double kLeftMaxUpwards = 504440; //474472
  public final double kLeftHalfway = kLeftMaxUpwards / 2;
  public final double kLeftSlightlyUpward = kLeftMaxUpwards / 4;

  public final double kRightMaxUpwards = 508601; //478895
  public final double kRightHalfway = kRightMaxUpwards / 2;
  public final double kRightSlightlyUpward = kRightMaxUpwards / 4;

  public final double kDownPowerCalibration = -0.3;   //has to go slower when calibrating to be more precise
  public final double kDownPowerNoCalibration = -0.9;
  
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
  public final NeutralMode kNeutral = NeutralMode.Brake;

  public final boolean kAlwaysZero = false;
}