package frc.robot.robotConstants.climbExtender;

public class TraversoClimbExtenderConstants {
  // extending motors (for climber)
  public final int kRightExtenderPort = 10;
  public final int kLeftExtenderPort = 11;

  // the arm's length in ticks
  public final double kExtenderMaxArmTicks = 509200;

  // tolerance allowed to the PID (inches)
  public final double kExtenderTolerance = 200;

  // motor clamps
  public final double kMotorClampOnLoad = .45;
  public final double kMotorClampOffLoad = .45;
  
  // locations
  public final double kMaxUpwards = kExtenderMaxArmTicks;
  public final double kMaxDownwards = 0;
  public final double kHalfway = kExtenderMaxArmTicks / 2;
  public final double kSlightlyUpward = kExtenderMaxArmTicks / 6;
  
  // off load PID constants
  public final double kP = 0.1;
  public final double kI = 0.00;
  public final double kD = 0.00;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final boolean kCoast = false;
}