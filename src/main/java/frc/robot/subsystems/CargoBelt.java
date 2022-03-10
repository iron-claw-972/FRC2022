/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.util.ControllerFactory;
import frc.robot.robotConstants.shooterBelt.DonkBeltConstants;
import edu.wpi.first.math.MathUtil;

public class CargoBelt extends SubsystemBase {

  DonkBeltConstants constants = new DonkBeltConstants();

  private final WPI_TalonFX m_CargoBeltMotor = ControllerFactory.createTalonFX(
    constants.kCargoBeltMotorPort , 
    constants.kSupplyCurrentLimit,
    constants.kSupplyTriggerThreshold, 
    constants.kSupplyTriggerDuration,
    constants.kCoast
  );

  private boolean enabled = false;
  private double motorPower = 0.0;

  @Override
  public void periodic() {
    if (enabled){
      setOutput(motorPower);
    }
  }
  
  public void setPower(double power) {
    motorPower = power;
  }
  
  public void setOutput(double motorPower) {
    m_CargoBeltMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
  }

  public void setStop() {
    motorPower = 0;
  }

  public void enable() {
    enabled = true;
  }

  public void disable() {
    enabled = false;
    setOutput(0);
  }

  public boolean isEnabled(){
    return enabled;
  }
}
