/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.shooterBelt.TraversoBeltConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class ShooterBelt extends SubsystemBase {

  TraversoBeltConstants constants = new TraversoBeltConstants();

  private final WPI_TalonFX m_ShooterBeltMotor = ControllerFactory.createTalonFX(constants.kShooterBeltMotorPort);

  private boolean enabled = false;
  private double motorPower = 0.0;

  public ShooterBelt() {

  }

  @Override
  public void periodic() {
    if (enabled){
      setOutput(motorPower);
    }
    else{
      setOutput(0);
    }
  }


  public void setOutput(double motorPower){
    m_ShooterBeltMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void setIntakeSpeed() {
    motorPower = constants.kIntakeSpeed;
  }

  public void setOuttakeSpeed() {
    motorPower = constants.kOuttakeSpeed;
  }

  public void setStop() {
    motorPower = 0;
  }

  public void enable(){
    enabled=true;
  }

  public void disable(){
    enabled=false;
  }

  public boolean reachedSetpoint(double targetSpeed) {
    return enabled;
  }

}
