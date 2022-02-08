package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheels extends SubsystemBase {

    private final CANSparkMax m_wheelsMotor = new CANSparkMax(ShooterConstants.kShooterWheelsMotorPort, MotorType.kBrushless);
    private final PIDController m_wheelsPID = new PIDController(ShooterConstants.kBottomMotorP, ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);

    public double wheelsMotorSpeed = 1.0;

    public ShooterWheels() {
    }

    @Override
    public void periodic() {
        m_wheelsMotor.set(m_wheelsPID.calculate(wheelsMotorSpeed));
    }

    public void setSpeed(double speed) {
        wheelsMotorSpeed = speed;
    }

    public void setIntakeSpeed() {  
        setSpeed(ShooterConstants.kShooterWheelsIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(ShooterConstants.kShooterWheelsBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(ShooterConstants.kShooterWheelsFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(ShooterConstants.kShooterWheelsFrontOutakeSpeed * ShooterConstants.kShooterWheelsFarMulti);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.kShooterWheelsBackOutakeSpeed * ShooterConstants.kShooterWheelsFarMulti);
    }

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        double encoderRate = m_wheelsMotor.getEncoder().getVelocity()*-1;
        if (encoderRate > targetSpeed - ShooterConstants.kShooterWheelsVelocityPIDTolerance && encoderRate < targetSpeed + ShooterConstants.kShooterWheelsVelocityPIDTolerance) {
            return true;
        }
        return false;
    }

}
