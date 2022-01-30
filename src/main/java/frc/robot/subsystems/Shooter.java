package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
    private final WPI_TalonFX m_motorFront = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPortFront);
    private final WPI_TalonFX m_motorBack = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPortBack);

    public Shooter() {}

    @Override
    public void periodic(){
        ballColor();
    }


    public void setSpeed(double speed) {
        m_motorFront.set(ControlMode.PercentOutput, speed);
        m_motorBack.set(ControlMode.PercentOutput, speed);

    }

    public void setBackOutakeSpeed(double speed) {
        m_motorBack.set(ControlMode.PercentOutput, speed);

    }

    public void setFrontOutakeSpeed(double speed) {
        m_motorFront.set(ControlMode.PercentOutput, speed);

    }

    public void setBackOutakeFarSpeed(double speed, double multiplier) {
        m_motorBack.set(ControlMode.PercentOutput, speed* multiplier);

    }

    public void setFrontOutakeFarSpeed(double speed, double multiplier) {
        m_motorFront.set(ControlMode.PercentOutput, speed* multiplier);

    }

    public void stop() {
        setSpeed(0);

    }

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();


    public void robotInit() {
        m_colorMatcher.addColorMatch(Color.kFirstBlue);
        m_colorMatcher.addColorMatch(Color.kFirstRed);
    }
    
    public String isColor(double r, double g, double b){
        if (b> 0.4) {
          return "blue";
        } else if (r > 0.4++++++) {
          return "red";
        }
        return "none";
    }  

    public String ballColor(){
        Color detectedColor = m_colorSensor.getColor();
        return isColor(detectedColor.red, detectedColor.green, detectedColor.blue);


    }


/*
Getters
.reachedSetpoint()
.containsBall()

*/

}
