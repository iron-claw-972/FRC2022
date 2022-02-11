package frc.robot.robotConstants.extenderArm;

public class TraversoExtenderArmConstants {
  // extending motors (for climber)
  public final int kRightExtenderPort = -1;
  public final int kLeftExtenderPort = -1;
  public final double kExtenderPower = 0.10;

  // the arm's max height - the arm's lowest height (max extension - max compression);
  public final double kExtenderMaxArmLength = 63 - 38; // at its highest, it's 63 inches, at its lowest its 38 inches

  // used to convert ticks to inches
  public final double kExtenderTicksPerRotation = 1/2048; // every rotation is 2048 ticks
  public final double kExtenderGearRatio = 20/1;
  public final double kExtenderInchesPerRotation = (Math.PI * 6); // 6 inches per rotation

  // Ticks Per Rotation * Gear Ratio * Inches Per Rotation = Tick Multiple
  public final double kExtenderTickMultiple = (
    kExtenderTicksPerRotation * kExtenderGearRatio * kExtenderInchesPerRotation
  );

  // Tick Multiple / Extension = Tick
  public final double kExtenderSetpoint = 0;

  // the allowed distance from the setpoint (IN TICKS)
  public final double kExtenderTolerance = 10;

  // the arm's length in ticks
  public final double kExtenderMaxArmTicks = kExtenderMaxArmLength / kExtenderTickMultiple;
}