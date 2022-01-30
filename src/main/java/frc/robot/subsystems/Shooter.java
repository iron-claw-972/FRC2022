package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Shooter extends SubsystemBase {
    
    public Shooter() {}

    private final WPI_TalonFX m_motorBack = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPortBack);
    private final WPI_TalonFX m_motorFront = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPortFront);

    @Override
    public void periodic() {
        System.out.println(reachedSetpoint(1));
    }

    public void setSpeed(double speed) {
        m_motorBack.set(ControlMode.PercentOutput, speed);
        m_motorFront.set(ControlMode.PercentOutput, speed);
    }

    public void setBackOutakeSpeed(double speed) {
        m_motorBack.set(ControlMode.PercentOutput, speed);
    }

    public void setFrontOutakeSpeed(double speed) {
        m_motorFront.set(ControlMode.PercentOutput, speed);
    }

    public void setFrontOutakeFarSpeed(double speed, double multiplier) {
        m_motorFront.set(ControlMode.PercentOutput, speed*multiplier);
    }

    public void setBackOutakeFarSpeed(double speed, double multiplier) {
        m_motorBack.set(ControlMode.PercentOutput, speed*multiplier);
    }

    public void stop() {
        setSpeed(0);
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

    public Boolean reachedSetpoint(double targetSpeed) {
        // returns true if the targetSpeed = current motor speed
        return false;
    }


}
