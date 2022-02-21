
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotConstants.colorSensor.TraversoColorSensorConstants;


import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class BallDetection extends SubsystemBase {

  TraversoColorSensorConstants constants = new TraversoColorSensorConstants();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  public void periodic() {
    SmartDashboard.putBoolean("blue", Color.kBlue == ballColor());
    SmartDashboard.putBoolean("red", Color.kRed == ballColor());
    SmartDashboard.putBoolean("has ball", containsBall());

  }

  public Color isColor(double red, double green, double blue) {
    if (blue > 0.4) {
      return Color.kBlue;
    } else if (red > 0.4) {
      return Color.kRed;
    }
    return null;
  }

  public Color ballColor() {
    Color detectedColor = m_colorSensor.getColor();
    return isColor(detectedColor.red, detectedColor.green, detectedColor.blue);
  }

  public boolean containsBall() {
    int ballProximity = m_colorSensor.getProximity();
    return (ballProximity > constants.kMinimumBallProximity);
  }

  public boolean ballShot() {
    return !containsBall();
  }

}