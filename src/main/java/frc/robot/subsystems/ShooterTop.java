package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterTop extends SubsystemBase {
    
    private final WPI_TalonFX m_topMotor = ControllerFactory.createTalonFX(ShooterConstants.kTopShooterMotorPort);
    private final PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopMotorP, ShooterConstants.kTopMotorI, ShooterConstants.kTopMotorD);
    private final TalonEncoder m_topShooterEncoder = new TalonEncoder(m_topMotor);

    public ShooterTop() {
        m_topShooterEncoder.setDistancePerPulse(ShooterConstants.kShooterMotorDistancePerPulse);
        m_topShooterEncoder.reset();
    }

    public void setSpeed(double speed) {
        m_topMotor.set(ControlMode.PercentOutput, m_topShooterPID.calculate(speed));
    }

    public void intake() {
        setSpeed(ShooterConstants.kTopIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(ShooterConstants.kTopBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(ShooterConstants.kTopFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(ShooterConstants.kTopFrontOutakeSpeed * ShooterConstants.kTopFarMultiplier);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.kTopBackOutakeSpeed * ShooterConstants.kTopFarMultiplier);
    }

    public void stop() {
        setSpeed(0);
    }

}
