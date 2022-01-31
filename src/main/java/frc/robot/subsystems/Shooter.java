package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class Shooter extends SubsystemBase {
    
    public Shooter() {}

    private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(1);
    private final PIDController shooterPID = new PIDController(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);

    @Override
    public void periodic() {
        System.out.println(containsBall());
    }

    public void setSpeed(double speed) {
        m_motor.set(ControlMode.PercentOutput, shooterPID.calculate(speed));
    }

    public void intake() {
        setSpeed(ShooterConstants.kIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(ShooterConstants.kBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(ShooterConstants.kFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(ShooterConstants.kFrontOutakeSpeed * ShooterConstants.kFarMultiplier);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.kBackOutakeSpeed * ShooterConstants.kFarMultiplier);
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

    public Boolean containsBall() {
        Integer ballProximity = m_colorSensor.getProximity();
        if (ballProximity > 1000) {
            return true;
        }
        return false;
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        // returns true if the targetSpeed = current motor speed
        return false;
    }


}
