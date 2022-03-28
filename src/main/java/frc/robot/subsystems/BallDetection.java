
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.util.Color;

public class BallDetection extends SubsystemBase {
  private final ColorSensorV3 m_colorSensor;

  private Color m_detectedColor;
  private int m_proximity;

  private Debouncer m_detectionDebouncer = new Debouncer(0.05, DebounceType.kRising);
  
  public BallDetection(){
    this(new ColorSensorV3(Constants.colorsensor.kI2cPort));
  }

  public BallDetection(ColorSensorV3 colorSensor) {
    m_colorSensor = colorSensor;
  }

  public void periodic() {
    m_detectedColor = m_colorSensor.getColor();
    m_proximity = m_colorSensor.getProximity();

    // boolean hasRedBall = hasRedBall();
    // boolean hasBlueBall = hasBlueBall();
    
    // SmartDashboard.putBoolean("Has Red Ball", hasRedBall);
    // SmartDashboard.putBoolean("Has Blue Ball", hasBlueBall);
    // SmartDashboard.putBoolean("Has Ball Securely", containsBallSecurely());
    // SmartDashboard.putBoolean("Has Ball", containsBall());
  }

  public boolean hasRedBall() {
    return containsBall() && m_detectedColor.red >= Constants.colorsensor.kRedBallThreshold;
  }

  public boolean hasBlueBall() {
    return containsBall() && m_detectedColor.blue >= Constants.colorsensor.kBlueBallThreshold;
  }

  public boolean hasRedBallSecure() {
    return containsBallSecurely() && m_detectedColor.red >= Constants.colorsensor.kRedBallThreshold;
  }

  public boolean hasBlueBallSecure() {
    return containsBallSecurely() && m_detectedColor.blue >= Constants.colorsensor.kBlueBallThreshold;
  }

  public boolean containsBall() {
    return m_detectionDebouncer.calculate(m_proximity >= Constants.colorsensor.kEdgeBallProximityThreshold);
  }

  public boolean containsBallSecurely() {
    return m_detectionDebouncer.calculate(m_proximity >= Constants.colorsensor.kSecureBallProximityThreshold);
  }
}