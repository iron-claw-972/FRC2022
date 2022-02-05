/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterWheelConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterWheel extends SubsystemBase {

    public ShooterWheel() {
        m_ShooterWheelEncoder.setDistancePerPulse(DriveConstants.kEncoderMetersPerPulse);
        m_ShooterWheelEncoder.reset();

    }    

    private final WPI_TalonFX m_ShooterWheelMotor = ControllerFactory.createTalonFX(ShooterWheelConstants.kShooterWheelMotorPort);
    private final TalonEncoder m_ShooterWheelEncoder = new TalonEncoder(m_ShooterWheelMotor);

    private final PIDController ShooterWheelPID = new PIDController(ShooterWheelConstants.kShooterWheelP, ShooterWheelConstants.kShooterWheelI, ShooterWheelConstants.kShooterWheelD);

    public static double motorSpeed = 1.0;

    @Override
    public void periodic() {
        if (reachedSetpoint(motorSpeed)){
            stop();    
        }else{
            m_ShooterWheelMotor.set(ControlMode.PercentOutput, ShooterWheelPID.calculate(motorSpeed));

        };
        
    }

    public void setSpeed(double newSpeed) {
        motorSpeed = newSpeed;
    }


    public void setBackOutakeSpeed() {
        motorSpeed = 1.4;
    }

    public void setFrontOutakeSpeed() {
        motorSpeed = 1.4;
    }

    public void setFrontOutakeFarSpeed() {
        motorSpeed = 1.4;
    }

    public void setBackOutakeFarSpeed() {
        motorSpeed = 1.4;
    }


    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        if (m_ShooterWheelEncoder.getRate() < targetSpeed + ShooterWheelConstants.kShooterWheelVelocityPIDTolerance &&
         m_ShooterWheelEncoder.getRate() > targetSpeed - ShooterWheelConstants.kShooterWheelVelocityPIDTolerance){
            return true;
        }
        return false;
    }

}
