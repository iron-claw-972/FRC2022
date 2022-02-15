package frc.robot.robotConstants.climbExtender;

public class TraversoClimbExtenderConstants {
  // extending motors (for climber)
  // TODO: Change these ports!
  public final int kRightExtenderPort = 2;
  public final int kLeftExtenderPort = 1;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  // TODO: Check the robot's arm length!
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
  public final double kMotorClamp = .1;
  
  // locations
  public final double kMaxUpwards = kExtenderMaxArmLength;
  public final double kMaxDownwards = 0;
  public final double kHalfway = kExtenderMaxArmLength / 2;
  public final double kSlightlyUpward = 6;
  
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
