/* top motor controls compliant wheels and belt*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.ControllerFactory;
import frc.robot.robotConstants.testWheel.MaciejTestWheelConstants;
import ctre_shims.TalonEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class TestWheel extends SubsystemBase {

  MaciejTestWheelConstants constants = new MaciejTestWheelConstants();

  private final WPI_TalonSRX m_TestWheelMotor = ControllerFactory.createTalonSRX(constants.kTestWheelMotorPort);

  private boolean enabled = false;
  private double motorPower = 0.0;

  public TestWheel() {

  }

  @Override
  public void periodic() {
    if (enabled){
      setOutput(motorPower);
    }
  }


  public void setOutput(double motorPower) {
    m_TestWheelMotor.set(ControlMode.PercentOutput, MathUtil.clamp(motorPower, -constants.kMotorClamp, constants.kMotorClamp));
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

  public void enable() {
    enabled=true;
  }

  public void disable() {
    enabled=false;
    setOutput(0);
  }

  public boolean reachedSetpoint(double targetSpeed) {
    return enabled;
  }

}
