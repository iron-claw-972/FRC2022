
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallDetectionConstants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class BallDetection extends SubsystemBase {

    public BallDetection() {

    }
    
    
    @Override
    public void periodic() {
        containsBall();
    }

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    public String isColor(double red, double green, double blue) {
        if (blue > 0.4) {
            return "blue";
        } else if (red > 0.4) {
            return "red";
        }
        return "none";
    }

    public String ballColor() {
        Color detectedColor = m_colorSensor.getColor();
        return isColor(detectedColor.red, detectedColor.green, detectedColor.blue);
    }

    public Boolean containsBall() {
        Integer ballProximity = m_colorSensor.getProximity();
        if (ballProximity > BallDetectionConstants.kMinimumBallProximity) {
            System.out.println("Object detected, proximity: " + ballProximity);
            return true;
        }
        System.out.println("No object detected, proximity: " + ballProximity);
        return false;
    }

}