/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterWheelConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheel extends SubsystemBase {    

    private final WPI_TalonFX m_ShooterWheelMotor = ControllerFactory.createTalonFX(ShooterWheelConstants.kShooterWheelMotorPort);
    private final TalonEncoder m_ShooterWheelEncoder = new TalonEncoder(m_ShooterWheelMotor);

    private final PIDController ShooterWheelPID = new PIDController(ShooterWheelConstants.kShooterWheelP, ShooterWheelConstants.kShooterWheelI, ShooterWheelConstants.kShooterWheelD);

    public static double motorSpeed = 1.0;

    public ShooterWheel() {
        m_ShooterWheelEncoder.setDistancePerPulse(ShooterWheelConstants.kEncoderMetersPerPulse);
        m_ShooterWheelEncoder.reset();
    }

    public void updatePID() {
        m_ShooterWheelMotor.set(ControlMode.PercentOutput, ShooterWheelPID.calculate(motorSpeed));
    }

    public void setSpeed(double newSpeed) {
        motorSpeed = newSpeed;
    }


    public void setBackOuttakeSpeed() {
        motorSpeed = ShooterWheelConstants.kBackOuttakeSpeed;
    }

    public void setFrontOuttakeSpeed() {
        motorSpeed = ShooterWheelConstants.kFrontOuttakeSpeed;
    }

    //TODO: Limelight integration
    /*
    public void setFrontOuttakeFarSpeed() {        
        motorSpeed = 1.0;
    }

    public void setBackOuttakeFarSpeed() {
        motorSpeed = 1.0;
    }
    */

    public void stop() {
        m_ShooterWheelMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean reachedSetpoint(double targetSpeed) {
        return (m_ShooterWheelEncoder.getRate() < targetSpeed + ShooterWheelConstants.kShooterWheelVelocityPIDTolerance &&
                m_ShooterWheelEncoder.getRate() > targetSpeed - ShooterWheelConstants.kShooterWheelVelocityPIDTolerance);
    }

}
