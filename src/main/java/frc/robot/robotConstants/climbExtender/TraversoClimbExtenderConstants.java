package frc.robot.robotConstants.climbExtender;

public class TraversoClimbExtenderConstants {
  // extending motors (for climber)
  public final int kRightExtenderPort = 11;
  public final int kLeftExtenderPort = 10;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  public final double kExtenderMaxArmLength = 63 - 38; // at its highest, it's 63 inches, at its lowest its 38 inches

  // used to convert ticks to inches
  public final double kExtenderTicksPerRotation = 1/2048; // every rotation is 2048 ticks
  public final double kExtenderGearRatio = 1/20; // gear ration of 20:1
  public final double kExtenderInchesPerRotation = (Math.PI * 1) / 1; // 1 pi inches per rotation

  // Ticks Per Rotation * Gear Ratio * Inches Per Rotation = Tick Multiple
  // Results as 0 when you calculate this in WPIlib, but it's definitely not 0
  public final double kExtenderTickMultiple =  0.00007669903; //(kExtenderTicksPerRotation * kExtenderGearRatio * kExtenderInchesPerRotation);

  // the arm's length in ticks
  public final double kExtenderMaxArmTicks = kExtenderMaxArmLength / kExtenderTickMultiple;

  // tolerance allowed to the PID (inches)
  public final double kExtenderTolerance = 1;

  // motor clamps
  public final double kMotorClampOnLoad = .35;
  public final double kMotorClampOffLoad = .35;
  
  // locations
  public final double kMaxUpwards = kExtenderMaxArmLength;
  public final double kMaxDownwards = 0;
  public final double kHalfway = kExtenderMaxArmLength / 2;
  public final double kSlightlyUpward = 6;
  
  // off load PID constants
  public final double kP = 0.1;
  public final double kI = 0.00;
  public final double kD = 0.00;

  public final double kSupplyCurrentLimit = 40;
  public final double kSupplyTriggerThreshold = 40;
  public final double kSupplyTriggerDuration = 0;
  public final boolean kCoast = false;
}