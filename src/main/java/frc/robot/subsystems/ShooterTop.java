package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
//SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

public class ShooterTop extends SubsystemBase {
    
    private final WPI_TalonFX m_topMotor = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPort);
    private final PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopMotorP, ShooterConstants.kTopMotorI, ShooterConstants.kTopMotorD);
    private final TalonEncoder m_topShooterEncoder = new TalonEncoder(m_topMotor);

    public ShooterTop() {
        m_topShooterEncoder.reset();
    }

    /*
    @Override
    public void periodic() {
        setSpeed(4);
        if (reachedSetpoint(80000) == true) {
            System.out.println("Motor has reached speed.");
        }
    } */

    public void setSpeed(double speed) {
        m_topMotor.set(ControlMode.PercentOutput, m_topShooterPID.calculate(speed));
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

}
