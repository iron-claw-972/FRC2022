package frc.robot.robotConstants.extenderArm;

public class TraversoExtenderArmConstants {
  // extending motors (for climber)
  // TODO: Change these ports!
  public final static int kRightExtenderPort = 2;
  public final static int kLeftExtenderPort = 1;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  // TODO: Check the robot's arm length!
  public final static double kExtenderMaxArmLength = 63 - 38; // at its highest, it's 63 inches, at its lowest its 38 inches

  // used to convert ticks to inches
  public final static double kExtenderTicksPerRotation = 1/2048; // every rotation is 2048 ticks
  public final static double kExtenderGearRatio = 1/20; // gear ration of 20:1
  public final static double kExtenderInchesPerRotation = (Math.PI * 1) / 1; // 1 pi inches per rotation

  // Ticks Per Rotation * Gear Ratio * Inches Per Rotation = Tick Multiple
  // Results as 0 when you calculate this in WPIlib, but it's definitely not 0
  public final static double kExtenderTickMultiple =  0.00007669903; //(kExtenderTicksPerRotation * kExtenderGearRatio * kExtenderInchesPerRotation);

  // the arm's length in ticks
  public final static double kExtenderMaxArmTicks = kExtenderMaxArmLength / kExtenderTickMultiple;

  // motor clamps
  public final static double kMotorClamp = .1;
  
  // locations
  public final static double kMaxUpwards = kExtenderMaxArmLength;
  public final static double kMaxDownwards = 0;
  public final static double kHalfway = kExtenderMaxArmLength / 2;
  public final static double kSlightlyUpward = 6;
}
