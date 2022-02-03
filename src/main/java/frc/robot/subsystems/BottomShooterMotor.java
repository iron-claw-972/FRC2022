/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.BottomShooterMotorConstants;

import com.revrobotics.ColorSensorV3;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class BottomShooterMotor extends SubsystemBase {

    public BottomShooterMotor() {
        m_bottomShooterEncoder.setDistancePerPulse(DriveConstants.kEncoderMetersPerPulse);
        m_bottomShooterEncoder.reset();

    }    

    private final WPI_TalonFX m_bottomMotor = ControllerFactory.createTalonFX(BottomShooterMotorConstants.kbottomShooterMotorPort);
    private final TalonEncoder m_bottomShooterEncoder = new TalonEncoder(m_bottomMotor);

    private final PIDController bottomShooterPID = new PIDController(BottomShooterMotorConstants.kbottomShooterP, BottomShooterMotorConstants.kbottomShooterI, BottomShooterMotorConstants.kbottomShooterD);


    public void setSpeed(double speed) {
        m_bottomMotor.set(ControlMode.PercentOutput, bottomShooterPID.calculate(speed));
    }

    public void intake() {
        setSpeed(BottomShooterMotorConstants.kbottomIntakeSpeed);
    }

    public void setBackOutakeSpeed() {
        setSpeed(BottomShooterMotorConstants.kbottomBackOutakeSpeed);
    }

    public void setFrontOutakeSpeed() {
        setSpeed(BottomShooterMotorConstants.kbottomFrontOutakeSpeed);
    }

    public void setFrontOutakeFarSpeed() {
        setSpeed(BottomShooterMotorConstants.kbottomFrontOutakeSpeed * BottomShooterMotorConstants.kbottomFarMultiplier);
    }

    public void setBackOutakeFarSpeed() {
        setSpeed(BottomShooterMotorConstants.kbottomBackOutakeSpeed * BottomShooterMotorConstants.kbottomFarMultiplier);
    }

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        if (m_bottomShooterEncoder.getRate() < targetSpeed + BottomShooterMotorConstants.kbottomShooterVelocityPIDTolerance &&
         m_bottomShooterEncoder.getRate() > targetSpeed - BottomShooterMotorConstants.kbottomShooterVelocityPIDTolerance){
            return true;
        }
        return false;
    }

}
