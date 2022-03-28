package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensorConstants {
  public final double kRedBallThreshold = 0.3;
  public final double kBlueBallThreshold = 0.3;
  public final int kEdgeBallProximityThreshold = 110;
  public final int kSecureBallProximityThreshold = 200;

  public final I2C.Port kI2cPort = I2C.Port.kMXP;
}