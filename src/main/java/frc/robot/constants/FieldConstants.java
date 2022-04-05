package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public final double distBallFromHub = Units.inchesToMeters(153);
  public final double distRobotStartFromHub = Units.inchesToMeters(101 - 8) - (Constants.drive.kRobotLength / 2.0);

  public final Translation2d hubPos = new Translation2d(8, 4.08);//Units.inchesToMeters(324), Units.inchesToMeters(162));
}
