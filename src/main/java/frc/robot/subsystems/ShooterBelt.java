/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterBeltConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterBelt extends SubsystemBase {

    private final WPI_TalonFX m_ShooterBeltMotor = ControllerFactory.createTalonFX(ShooterBeltConstants.kShooterBeltMotorPort);
    private final TalonEncoder m_ShooterBeltEncoder = new TalonEncoder(m_ShooterBeltMotor);

    private final PIDController ShooterBeltPID = new PIDController(ShooterBeltConstants.kShooterBeltP, ShooterBeltConstants.kShooterBeltI, ShooterBeltConstants.kShooterBeltD);

    public static double motorSpeed = 0.0;

    public ShooterBelt() {
        m_ShooterBeltEncoder.setDistancePerPulse(ShooterBeltConstants.kEncoderMetersPerPulse);
        m_ShooterBeltEncoder.reset();
    }

    public void updatePID() {
        m_ShooterBeltMotor.set(ControlMode.PercentOutput, ShooterBeltPID.calculate(motorSpeed));
    }

    public void setSpeed(double newSpeed) {
        motorSpeed = newSpeed;
    }

    public void setIntakeSpeed() {
        motorSpeed = ShooterBeltConstants.kIntakeSpeed;
    }

    public void setOuttakeSpeed() {
        motorSpeed = ShooterBeltConstants.kOuttakeSpeed;
    }

    public void stop() {
        m_ShooterBeltMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean reachedSetpoint(double targetSpeed) {
        return (m_ShooterBeltEncoder.getRate() < targetSpeed + ShooterBeltConstants.kShooterBeltVelocityPIDTolerance &&
                m_ShooterBeltEncoder.getRate() > targetSpeed - ShooterBeltConstants.kShooterBeltVelocityPIDTolerance);
    }

}
