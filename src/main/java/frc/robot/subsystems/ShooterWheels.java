package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheels extends SubsystemBase {
    
    private final WPI_TalonFX m_wheelsMotor = ControllerFactory.createTalonFX(ShooterConstants.kShooterWheelsMotorPort);
    private final PIDController m_wheelsPID = new PIDController(ShooterConstants.kTopMotorP, ShooterConstants.kTopMotorI, ShooterConstants.kTopMotorD);
    private final TalonEncoder m_wheelsEncoder = new TalonEncoder(m_wheelsMotor);

    public double wheelsMotorSpeed = 1.0;

    public ShooterWheels() {
        m_wheelsEncoder.setDistancePerPulse(ShooterConstants.kShooterMotorDistancePerPulse);
        m_wheelsEncoder.reset();
    }

    @Override
    public void periodic() {
        m_wheelsMotor.set(ControlMode.PercentOutput, m_wheelsPID.calculate(wheelsMotorSpeed));
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
        double encoderRate = m_wheelsEncoder.getRate()*-1;
        if (encoderRate > targetSpeed - ShooterConstants.kShooterWheelsVelocityPIDTolerance && encoderRate < targetSpeed + ShooterConstants.kShooterWheelsVelocityPIDTolerance) {
            return true;
        }
        return false;
    }

}
