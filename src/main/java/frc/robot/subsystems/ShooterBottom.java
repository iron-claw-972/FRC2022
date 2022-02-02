package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterBottom extends SubsystemBase {

    private final WPI_TalonFX m_bottomMotor = ControllerFactory.createTalonFX(ShooterConstants.kBottomShooterMotorPort);
    private final PIDController m_bottomShooterPID = new PIDController(ShooterConstants.kBottomMotorP, ShooterConstants.kBottomMotorI, ShooterConstants.kBottomMotorD);
    private final TalonEncoder m_bottomShooterEncoder = new TalonEncoder(m_bottomMotor);

    public ShooterBottom() {
        m_bottomShooterEncoder.setDistancePerPulse(ShooterConstants.kShooterMotorDistancePerPulse);
        m_bottomShooterEncoder.reset();
    }

    public void setSpeed(double speed) {
        m_bottomMotor.set(ControlMode.PercentOutput, m_bottomShooterPID.calculate(speed));
    }

    public void intake() {
        setSpeed(ShooterConstants.kBottomIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(ShooterConstants.kBottomBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(ShooterConstants.kBottomFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(ShooterConstants.kBottomFrontOutakeSpeed * ShooterConstants.kBottomFarMultiplier);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(ShooterConstants.kBottomBackOutakeSpeed * ShooterConstants.kBottomFarMultiplier);
    }

    public void stop() {
        setSpeed(0);
    }

}
