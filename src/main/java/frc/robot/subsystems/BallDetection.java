
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robotConstants.colorSensor.TraversoColorSensorConstants;


import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class BallDetection extends SubsystemBase {

  TraversoColorSensorConstants constants = new TraversoColorSensorConstants();

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  
  // private Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kMillimeters, RangeProfile.kHighAccuracy);

  public BallDetection(){
  }

  public void periodic() {

    // distanceSensor.setEnabled(true);
    // distanceSensor.setAutomaticMode(true);
    // SmartDashboard.putNumber("distance", distanceSensor.getRange());
    // System.out.println(distanceSensor.getRange());
    
    // distanceSensor.
    // SmartDashboard.putBoolean("blue", Color.kBlue == ballColor());
    // SmartDashboard.putBoolean("red", Color.kRed == ballColor());
    // SmartDashboard.putBoolean("has ball", containsBall());


    SmartDashboard.putNumber("proximity",m_colorSensor.getProximity());
    System.out.println(m_colorSensor.getProximity());

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