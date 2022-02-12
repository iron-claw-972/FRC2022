package frc.robot.robotConstants.extenderArm;

public class TraversoExtenderArmConstants {
  // extending motors (for climber)
  // TODO: Change these ports!
  public final int kRightExtenderPort = 2;
  public final int kLeftExtenderPort = 1;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  // TODO: Check the robot's arm length!
  public final double kExtenderMaxArmLength = 63 - 38; // at its highest, it's 63 inches, at its lowest its 38 inches

  // used to convert ticks to inches
  public final double kExtenderTicksPerRotation = 1/2048; // every rotation is 2048 ticks
  public final double kExtenderGearRatio = 20/1;
  public final double kExtenderInchesPerRotation = (Math.PI * 6); // 6 pi inches per rotation

  // Ticks Per Rotation * Gear Ratio * Inches Per Rotation = Tick Multiple
  // Results as 0 when you calculate this in WPI lib, but it's definitely not 0
  public final double kExtenderTickMultiple = 0.184077695; //(kExtenderTicksPerRotation * kExtenderGearRatio * kExtenderInchesPerRotation);

  // the arm's length in ticks
  public final double kExtenderMaxArmTicks = kExtenderMaxArmLength / kExtenderTickMultiple;

  // motor clamps
  public final double kMotorClamp = .1;
}
