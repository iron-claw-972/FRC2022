package frc.robot.robotConstants.extenderArm;

public class TraversoExtenderArmConstants {
  // extending motors (for climber)
  // TODO: Change these ports!
  public static final int kRightExtenderPort = 2;
  public static final int kLeftExtenderPort = 1;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  // TODO: Check the robot's arm length!
  public static final double kExtenderMaxArmLength = 63 - 38; // at its highest, it's 63 inches, at its lowest its 38 inches

  // used to convert ticks to inches
  public static final double kExtenderTicksPerRotation = 1/2048; // every rotation is 2048 ticks
  public static final double kExtenderGearRatio = 1/20; // gear ration of 20:1
  public static final double kExtenderInchesPerRotation = (Math.PI * 1) / 1; // 1 pi inches per rotation

  // Ticks Per Rotation * Gear Ratio * Inches Per Rotation = Tick Multiple
  // Results as 0 when you calculate this in WPIlib, but it's definitely not 0
  public static final double kExtenderTickMultiple =  0.00007669903; //(kExtenderTicksPerRotation * kExtenderGearRatio * kExtenderInchesPerRotation);

  // the arm's length in ticks
  public static final double kExtenderMaxArmTicks = kExtenderMaxArmLength / kExtenderTickMultiple;

  // motor clamps
  public static final double kMotorClamp = .1;
  
  // locations
  public static final double kMaxUpwards = kExtenderMaxArmLength;
  public static final double kMaxDownwards = 0;
  public static final double kHalfway = kExtenderMaxArmLength / 2;
  public static final double kSlightlyUpward = 6;
}
