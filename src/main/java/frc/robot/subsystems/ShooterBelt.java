/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterBeltConstants;

import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ShooterBelt extends SubsystemBase {

    public ShooterBelt() {
        m_ShooterBeltEncoder.setDistancePerPulse(DriveConstants.kEncoderMetersPerPulse);
        m_ShooterBeltEncoder.reset();

    }    

    private final WPI_TalonFX m_ShooterBeltMotor = ControllerFactory.createTalonFX(ShooterBeltConstants.kShooterBeltMotorPort);
    private final TalonEncoder m_ShooterBeltEncoder = new TalonEncoder(m_ShooterBeltMotor);

    private final PIDController ShooterBeltPID = new PIDController(ShooterBeltConstants.kShooterBeltP, ShooterBeltConstants.kShooterBeltI, ShooterBeltConstants.kShooterBeltD);

    public static double motorSpeed = 1.0;

    @Override
    public void periodic() {
        if (reachedSetpoint(motorSpeed)){
            stop();    
        }else{
            m_ShooterBeltMotor.set(ControlMode.PercentOutput, ShooterBeltPID.calculate(motorSpeed));

        };
        
    }

    public void setSpeed(double newSpeed) {
        motorSpeed = newSpeed;
    }

    public void setIntakeSpeed() {
        motorSpeed = 2.0;
    }

    public void setOutakeSpeed() {
        motorSpeed = 2.0;
    }

    /*
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
    */

    public void stop() {
        setSpeed(0);
    }

    public Boolean reachedSetpoint(double targetSpeed) {
        if (m_ShooterBeltEncoder.getRate() < targetSpeed + ShooterBeltConstants.kShooterBeltVelocityPIDTolerance &&
         m_ShooterBeltEncoder.getRate() > targetSpeed - ShooterBeltConstants.kShooterBeltVelocityPIDTolerance){
            return true;
        }
        return false;
    }

}
