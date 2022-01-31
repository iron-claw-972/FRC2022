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
<<<<<<< HEAD
    private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPort);

=======
    
>>>>>>> e25ec55ae945e0ada692c874cf3f4b210ce8d4e9
    public Shooter() {}

    private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(1);
    private final PIDController shooterPID = new PIDController(ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);

    @Override
    public void periodic() {
        System.out.println(containsBall());
    }

    public void setSpeed(double speed) {
<<<<<<< HEAD
        m_motor.set(ControlMode.PercentOutput, speed);

    }

    public void setBackOutakeSpeed() {
        setSpeed(ShooterConstants.speed);

    }

    public void setFrontkOutakeSpeed() {
        setSpeed(ShooterConstants.speed);

    }

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.speed* ShooterConstants.multiplier);

    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(ShooterConstants.speed* ShooterConstants.multiplier);
=======
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
>>>>>>> e25ec55ae945e0ada692c874cf3f4b210ce8d4e9

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.kBackOutakeSpeed * ShooterConstants.kFarMultiplier);
    }

    public void intake(){
        setSpeed(ShooterConstants.speed*-1);
    }

    public void stop() {
        setSpeed(0);
    }

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

<<<<<<< HEAD
    public void robotInit() {
        m_colorMatcher.addColorMatch(Color.kFirstBlue);
        m_colorMatcher.addColorMatch(Color.kFirstRed);
    }
    
    public String isColor(double r, double g, double b){
        if (b> 0.4) {
          return "blue";
        } else if (r > 0.4) {
          return "red";
=======
    public String isColor(double red, double green, double blue) {
        if (blue > 0.4) {
            return "blue";
        } else if (red > 0.4) {
            return "red";
>>>>>>> e25ec55ae945e0ada692c874cf3f4b210ce8d4e9
        }
        return "none";
    }

    public String ballColor() {
        Color detectedColor = m_colorSensor.getColor();
        return isColor(detectedColor.red, detectedColor.green, detectedColor.blue);
<<<<<<< HEAD

    }

    

=======
    }

    public Boolean containsBall() {
        Integer ballProximity = m_colorSensor.getProximity();
        if (ballProximity > 1000) {
            return true;
        }
        return false;
    }
>>>>>>> e25ec55ae945e0ada692c874cf3f4b210ce8d4e9

    public Boolean reachedSetpoint(double targetSpeed) {
        // returns true if the targetSpeed = current motor speed
        return false;
    }


}
