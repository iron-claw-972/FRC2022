package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.Constants.ShooterConstants;;

public class Shooter {
    private final WPI_TalonFX m_motor = ControllerFactory.createTalonFX(ShooterConstants.kShooterMotorPortTop);

    public void setSpeed(double speed) {
        m_motor.set(ControlMode.PercentOutput, speed);
    }
    /*.setSpeed(double speed) - done
.setBackOutakeSpeed() 
.setFrontOutakeSpeed()
.setBackOutakeFarSpeed()
.setFrontOutakeFarSpeed()
.stop()
Getters
.reachedSetpoint()
.containsBall()
.ballColor()

*/

}
