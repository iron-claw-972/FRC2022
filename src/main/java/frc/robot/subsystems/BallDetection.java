
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotConstants.colorSensor.TraversoColorSensorConstants;


import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class BallDetection extends SubsystemBase {

  TraversoColorSensorConstants constants = new TraversoColorSensorConstants();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private Color detectedColor;
  private int proximity;
  
  public BallDetection(){

  }

  public void periodic() {
    detectedColor = m_colorSensor.getColor();
    proximity = m_colorSensor.getProximity();

z  }

  public boolean hasRedBall() {
    return containsBall() && detectedColor.red >= constants.kRedBallThreshold;
  }

  public boolean hasBlueBall() {
    return containsBall() && detectedColor.blue >= constants.kBlueBallThreshold;
  }

  public boolean hasRedBallSecure() {
    return containsBallSecurely() && detectedColor.red >= constants.kRedBallThreshold;
  }

  public boolean hasBlueBallSecure() {
    return containsBallSecurely() && detectedColor.blue >= constants.kBlueBallThreshold;
  }

  public boolean containsBall() {
    return (proximity >= constants.kEdgeBallProximityThreshold);
  }

  public boolean containsBallSecurely() {
    return (proximity >= constants.kSecureBallProximityThreshold);
  }
}