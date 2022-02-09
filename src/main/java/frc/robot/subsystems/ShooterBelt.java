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

public class ShooterBelt extends SubsystemBase {

    private final CANSparkMax m_beltMotor = new CANSparkMax(ShooterConstants.kShooterBeltMotorPort, MotorType.kBrushless);
    private final PIDController m_beltPID = new PIDController(ShooterConstants.kBottomMotorP, ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);

    public double beltMotorSpeed = 1.0;

    public ShooterBelt() {
    }

    @Override
    public void periodic() {
        m_beltMotor.set(m_beltPID.calculate(beltMotorSpeed));
    }

    public void setSpeed(double speed) {
        beltMotorSpeed = speed;
    }

    public void setIntakeSpeed() {
        setSpeed(ShooterConstants.kShooterBeltIntakeSpeed);
    }

    public void setOutakeSpeed() {
        setSpeed(ShooterConstants.kShooterBeltOutakeSpeed);
    }

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        double encoderRate = m_beltMotor.getEncoder().getVelocity()*-1;
        if (encoderRate > targetSpeed - ShooterConstants.kShooterBeltVelocityPIDTolerance && encoderRate < targetSpeed + ShooterConstants.kShooterBeltVelocityPIDTolerance) {
            return true;
        }
        return false;
    }

}
