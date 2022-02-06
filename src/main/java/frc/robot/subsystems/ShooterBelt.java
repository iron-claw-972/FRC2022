package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterBelt extends SubsystemBase {

    private final WPI_TalonFX m_beltMotor = ControllerFactory.createTalonFX(ShooterConstants.kShooterBeltMotorPort);
    private final PIDController m_beltPID = new PIDController(ShooterConstants.kBottomMotorP, ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);
    private final TalonEncoder m_beltEncoder = new TalonEncoder(m_beltMotor);

    public double beltMotorSpeed = 1.0;

    public ShooterBelt() {
        m_beltEncoder.setDistancePerPulse(ShooterConstants.kShooterMotorDistancePerPulse);
        m_beltEncoder.reset();
    }

    @Override
    public void periodic() {
        m_beltMotor.set(ControlMode.PercentOutput, m_beltPID.calculate(beltMotorSpeed));
    }

    public void setSpeed(double speed) {
        beltMotorSpeed = speed;
    }

    public void setIntakeSpeed() {
        setSpeed(ShooterConstants.kShooterBeltIntakeSpeed);
    }

    public void setOutakeSpeed() {
        setSpeed(ShooterConstants.kShooterBeltIntakeSpeed);
    }

    // public void setBackOutakeSpeed() {
    //     setSpeed(ShooterConstants.kShooterBeltBackOutakeSpeed);
    // }

    // public void setFrontOutakeSpeed() {
    //     setSpeed(ShooterConstants.kShooterBeltFrontOutakeSpeed);
    // }

    // public void setFrontOutakeFarSpeed() {
    //     setSpeed(ShooterConstants.kShooterBeltFrontOutakeSpeed * ShooterConstants.kShooterBeltFarMulti);
    // }

    // public void setBackOutakeFarSpeed() {
    //     setSpeed(ShooterConstants.kShooterBeltBackOutakeSpeed * ShooterConstants.kShooterBeltFarMulti);
    // }

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        double encoderRate = m_beltEncoder.getRate()*-1;
        if (encoderRate > targetSpeed - ShooterConstants.kShooterBeltVelocityPIDTolerance && encoderRate < targetSpeed + ShooterConstants.kShooterBeltVelocityPIDTolerance) {
            return true;
        }
        return false;
    }

}
