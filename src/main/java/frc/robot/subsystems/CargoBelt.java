/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.Constants;
import frc.robot.util.ControllerFactory;
import edu.wpi.first.math.MathUtil;

public class CargoBelt extends SubsystemBase {
  private final WPI_TalonFX m_cargoBeltMotor = ControllerFactory.createTalonFX(
    Constants.belt.kCargoBeltMotorPort , 
    Constants.belt.kSupplyCurrentLimit,
    Constants.belt.kSupplyTriggerThreshold, 
    Constants.belt.kSupplyTriggerDuration,
    Constants.belt.kNeutral
  );

  private boolean m_enabled = false;
  private double m_motorPower = 0.0;

  @Override
  public void periodic() {
    if (m_enabled){
      setOutput(m_motorPower);
    }
  }
  
  public void setPower(double power) {
    m_motorPower = power;
  }
  
  public void setOutput(double motorPower) {
    m_cargoBeltMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -Constants.belt.kMotorClamp, Constants.belt.kMotorClamp));
  }

  public void setStop() {
    m_motorPower = 0;
  }

  public void enable() {
    m_enabled = true;
  }

  public void disable() {
    m_enabled = false;
    setOutput(0);
  }

  public boolean isEnabled(){
    return m_enabled;
  }
}
