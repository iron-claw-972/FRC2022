
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;

import com.revrobotics.ColorSensorV3;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Sensors extends SubsystemBase {

    public Sensors() {

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
        if (ballProximity > SensorConstants.kMinimumBallProximity) {
            System.out.println("Object detected, proximity: " + ballProximity);
            return true;
        }
        System.out.println("No object detected, proximity: " + ballProximity);
        return false;
    }

}