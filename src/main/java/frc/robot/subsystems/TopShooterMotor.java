/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TopShooterMotorConstants;

import com.revrobotics.ColorSensorV3;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class TopShooterMotor extends SubsystemBase {

    public TopShooterMotor() {
        m_topShooterEncoder.setDistancePerPulse(DriveConstants.kEncoderMetersPerPulse);
        m_topShooterEncoder.reset();

    }    

    private final WPI_TalonFX m_topMotor = ControllerFactory.createTalonFX(TopShooterMotorConstants.ktopShooterMotorPort);
    private final TalonEncoder m_topShooterEncoder = new TalonEncoder(m_topMotor);

    private final PIDController topShooterPID = new PIDController(TopShooterMotorConstants.ktopShooterP, TopShooterMotorConstants.ktopShooterI, TopShooterMotorConstants.ktopShooterD);


    public void setSpeed(double speed) {
        m_topMotor.set(ControlMode.PercentOutput, topShooterPID.calculate(speed));
    }

    public void intake() {
        setSpeed(TopShooterMotorConstants.ktopIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(TopShooterMotorConstants.ktopBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(TopShooterMotorConstants.ktopFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(TopShooterMotorConstants.ktopFrontOutakeSpeed * TopShooterMotorConstants.ktopFarMultiplier);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(TopShooterMotorConstants.ktopBackOutakeSpeed * TopShooterMotorConstants.ktopFarMultiplier);
    }

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        if (m_topShooterEncoder.getRate() < targetSpeed + TopShooterMotorConstants.ktopShooterVelocityPIDTolerance &&
         m_topShooterEncoder.getRate() > targetSpeed - TopShooterMotorConstants.ktopShooterVelocityPIDTolerance){
            return true;
        }
        return false;
    }

}
